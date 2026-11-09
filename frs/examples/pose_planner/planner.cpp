#include "planner.hpp"
#include "movetools.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <stdexcept>

using Json = nlohmann::json;

namespace {
// FBA 的三个入口使用 C 链接名，但返回 C++ 对象；不能直接用 Python ctypes 调 plan。
// 本适配层把 JSON 转为同版本 DTO，再在 C++ 中调用虚函数。
class Plugin {
public:
    explicit Plugin(const fba::Config& config) {
        library_ = dlopen(config.movetools_plugin.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!library_) throw std::runtime_error(std::string("MoveTools plugin: ") + dlerror());
        std::string error;
        try {
            const auto version = reinterpret_cast<fba::PlannerVersion>(dlsym(library_, "fba_movetools_version"));
            const auto create = reinterpret_cast<fba::PlannerCreate>(dlsym(library_, "fba_movetools_create"));
            destroy_ = reinterpret_cast<fba::PlannerDestroy>(dlsym(library_, "fba_movetools_destroy"));
            if (!version || !create || !destroy_ || version() != fba::MOVETOOLS_PLUGIN_ABI)
                throw std::runtime_error("MoveTools plugin ABI mismatch (requires 3)");
            planner_ = create(&config);
            if (!planner_) throw std::runtime_error("MoveTools plugin initialization failed");
        } catch (const std::exception& exception) { error = exception.what(); }
        catch (...) { error = "MoveTools plugin initialization failed"; }
        // 先销毁插件抛出的异常对象，再卸载其代码和 RTTI。
        if (!planner_) { dlclose(library_); library_ = nullptr; throw std::runtime_error(error); }
    }
    ~Plugin() {
        if (planner_) destroy_(planner_); // 必须在 dlclose 前销毁插件对象。
        if (library_) dlclose(library_);
    }
    Plugin(const Plugin&) = delete;
    Plugin& operator=(const Plugin&) = delete;
    fba::JointTrajectory plan(const fba::Command& command, const fba::RobotSnapshot& initial) {
        std::string error;
        try { return planner_->plan(command, initial); }
        catch (const std::exception& exception) { error = exception.what(); }
        catch (...) { error = "MoveTools planning failed"; }
        // 把插件异常转为本库的异常后，调用方才可以安全卸载插件。
        throw std::runtime_error(error);
    }
private:
    void* library_{};
    fba::MotionPlanner* planner_{};
    fba::PlannerDestroy destroy_{};
};

std::map<std::string, fba::JointGroupState> groups(const Json& input) {
    std::map<std::string, fba::JointGroupState> result;
    for (const auto& group : input) {
        const auto name = group.at("group_name").get<std::string>();
        fba::JointGroupState state{group.at("joint_position").get<std::vector<double>>(),
            group.value("joint_velocity", std::vector<double>{}),
            group.value("joint_effort", std::vector<double>{})};
        for (const auto* values : {&state.position, &state.velocity, &state.effort})
            for (double value : *values)
                if (!std::isfinite(value)) throw std::runtime_error("non-finite planning state: " + name);
        if (!result.emplace(name, std::move(state)).second)
            throw std::runtime_error("duplicate planning state group: " + name);
    }
    // MoveTools 需要整机初始姿态，不能只传准备移动的右臂。
    for (const auto& [name, count] : std::map<std::string, std::size_t>{
             {"left_leg",6}, {"right_leg",6}, {"waist",3}, {"head",2},
             {"left_manipulator",7}, {"right_manipulator",7}}) {
        const auto found = result.find(name);
        if (found == result.end() || found->second.position.size() != count)
            throw std::runtime_error("incomplete GR3 planning state: " + name);
    }
    return result;
}

Json plan(const Json& input) {
    fba::Config config;
    // Atom 传入三项绝对路径：插件、规划参数、机器人模型配置。
    config.movetools_plugin = input.at("plugin").get<std::string>();
    config.planning_config_dir = input.at("planning").get<std::string>();
    config.robot_config_root = input.at("robot_config").get<std::string>();
    config.control_hz = 30;
    config.max_duration_s = 20;
    config.arm_groups["right_manipulator"] = "right_end_effector_link";
    for (const auto* path : {&config.movetools_plugin, &config.planning_config_dir, &config.robot_config_root})
        if (!std::filesystem::path(*path).is_absolute()) throw std::runtime_error("planner paths must be absolute");
    fba::RobotSnapshot initial;
    initial.measured = groups(input.at("joint_state"));
    initial.command = groups(input.at("joint_cmd"));
    fba::Command command;
    command.kind = fba::CommandKind::MoveArm;
    command.group = "right_manipulator";
    command.joint_target = input.at("target").get<std::vector<double>>();
    command.speed_ratio = 0.2; // 与 FBA move_pose 的机械臂关节目标路径一致。
    if (command.joint_target.size() != 7) throw std::runtime_error("right arm target requires seven positions");
    Plugin plugin(config);
    const auto trajectory = plugin.plan(command, initial); // 只计算，不申请资源、不发命令。
    if (trajectory.group != command.group || trajectory.positions.empty() ||
        trajectory.positions.size() > 601 || !std::isfinite(trajectory.dt) ||
        std::abs(trajectory.dt - 1.0 / config.control_hz) > 1e-9 ||
        trajectory.lower.size() != 7 || trajectory.upper.size() != 7)
        throw std::runtime_error("invalid or over-budget MoveTools trajectory");
    // 插件结果也需要校验。首帧和每个相邻帧最多变化 0.15 rad。
    auto previous = initial.command.at(command.group).position;
    for (const auto& frame : trajectory.positions) {
        if (frame.size() != 7) throw std::runtime_error("MoveTools frame dimension mismatch");
        for (std::size_t i = 0; i < frame.size(); ++i)
            if (!std::isfinite(frame[i]) || !std::isfinite(trajectory.lower[i]) ||
                !std::isfinite(trajectory.upper[i]) || trajectory.lower[i] >= trajectory.upper[i] ||
                frame[i] < trajectory.lower[i] || frame[i] > trajectory.upper[i] ||
                std::abs(frame[i] - previous[i]) > 0.15)
                throw std::runtime_error("MoveTools frame violates limits or step budget");
        previous = frame;
    }
    for (std::size_t i = 0; i < previous.size(); ++i)
        if (std::abs(previous[i] - command.joint_target[i]) > 0.03)
            throw std::runtime_error("MoveTools trajectory does not end at target");
    return {{"ok",true}, {"dt",trajectory.dt}, {"positions",trajectory.positions},
            {"lower",trajectory.lower}, {"upper",trajectory.upper}};
}
}

extern "C" int thumbs_up_plan(const char* input, char* output, std::size_t capacity) noexcept {
    if (!input || !output || !capacity) return 2;
    try {
        Json result;
        int code = 0;
        try { result = plan(Json::parse(input)); }
        catch (const std::exception& error) { result = {{"ok",false}, {"error",error.what()}}; code = 1; }
        catch (...) { result = {{"ok",false}, {"error","MoveTools planning failed"}}; code = 1; }
        const auto text = result.dump();
        if (text.size() + 1 > capacity) return 2;
        std::memcpy(output, text.c_str(), text.size() + 1);
        return code;
    } catch (...) { return 2; }
}
