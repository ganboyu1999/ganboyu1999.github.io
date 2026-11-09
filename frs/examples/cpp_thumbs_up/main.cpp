#include <frs/frs.hpp>  // FRS 生命周期、Topic 读取、命令和报告。
#include <gr_interface/atom/GrAurora.hpp>  // 公共消息和 CDR codec。
#include "planner.hpp"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

namespace sdk = fourier::aurora_sdk;
using Value = frs::json::Value;
using Clock = std::chrono::steady_clock;
using Message = gr_interface::msg::RobotControlGroupState;
constexpr auto topic = "/motion_ctrl/aurora/control_group_state";
constexpr auto groupName = "right_end_effector";
constexpr auto period = std::chrono::microseconds(33333);
const std::vector<double> armGoal{-0.35, -0.25, 0, -1.25, 0, 0, 0};
const std::vector<double> handGoal{1.70, 1.70, 1.70, 1.70, 0.15, 0.80};
using Json = nlohmann::json;
using Positions = std::map<std::string, std::vector<double>>;

struct Profile {
    // FDH-6R 右手的六个接口：食指、中指、无名指、小指、拇指俯仰、拇指偏航。
    // 数组下标就是命令/反馈下标；这些值是 FHM 手部接口角度，单位 rad。
    std::vector<double> lower{0.17, 0.17, 0.17, 0.17, 0.12, 0.0};
    std::vector<double> upper{1.78, 1.78, 1.78, 1.78, 1.28, 1.68};
    double tolerance{0.03}; // 右手实测到位容差。
};

void checked(const std::vector<double>& position, const Profile& profile) {
    // 输入、插件目标和每个输出帧都验证维度、NaN/Inf 和接口坐标限位。
    if (position.size() != profile.lower.size()) throw std::runtime_error("hand position dimension mismatch");
    for (std::size_t i = 0; i < position.size(); ++i)
        if (!std::isfinite(position[i]) || position[i] < profile.lower[i] - 1e-6 || position[i] > profile.upper[i] + 1e-6)
            throw std::runtime_error("hand position outside limits or non-finite");
}

struct Feedback {
    frs::atom::TopicSample<Message> sample;
    Positions measured;
    std::vector<double> armCommand;
    Json message;
};

Feedback readFeedback(const Profile& profile) {
    // get 从共享缓存解码完整消息；IDL 的 getter 并不是 FRS 自定义方法。
    auto sample = frs::get<Message>(topic);
    if (!sample.frs.available || sample.frs.quality != "fresh" || sample.frs.age_ns < 0 || sample.frs.age_ns > 100'000'000)
        throw std::runtime_error("pose feedback is unavailable or older than 100 ms");
    Json message;
    const auto convert = [](const auto& groups) {
        Json result = Json::array();
        for (const auto& group : groups) {
            for (const auto* values : {&group.joint_position(), &group.joint_velocity(), &group.joint_effort()})
                for (double value : *values)
                    if (!std::isfinite(value)) throw std::runtime_error("non-finite control group state");
            result.push_back({{"group_name",group.group_name()}, {"joint_position",group.joint_position()},
                {"joint_velocity",group.joint_velocity()}, {"joint_effort",group.joint_effort()}});
        }
        return result;
    };
    message["joint_state"] = convert(sample.msg().joint_state());
    message["joint_cmd"] = convert(sample.msg().joint_cmd());
    const auto extract = [&](const char* field, const std::string& name, std::size_t count) {
        std::vector<double> values;
        bool found = false;
        for (const auto& group : message.at(field)) {
            if (group.at("group_name") != name) continue;
            if (found) throw std::runtime_error("duplicate feedback: " + name);
            found = true; values = group.at("joint_position").get<std::vector<double>>();
            for (const auto* key : {"joint_velocity", "joint_effort"})
                if (!group.at(key).empty() && group.at(key).size() != count)
                    throw std::runtime_error("invalid feedback dimensions: " + name);
        }
        if (!found || values.size() != count) throw std::runtime_error("missing or invalid feedback: " + name);
        return values;
    };
    Positions measured{{"right_manipulator",extract("joint_state","right_manipulator",7)},
                       {groupName,extract("joint_state",groupName,6)}};
    checked(measured.at(groupName), profile);
    auto command = extract("joint_cmd","right_manipulator",7);
    return {std::move(sample),std::move(measured),std::move(command),std::move(message)};
}

Json planArm(Json message) {
    // 本机配置只提供模型和规划参数，固定目标直接写在 Atom 内。
    message["plugin"] = std::filesystem::absolute("plugins/libfba_movetools.so").string();
    message["planning"] = std::filesystem::absolute("planning/gr3v2_2_4").string();
    message["robot_config"] = "/opt/fftai/fourier_config_center";
    message["target"] = armGoal;
    const auto input = message.dump();
    std::vector<char> output(2*1024*1024);
    const auto code = thumbs_up_plan(input.c_str(), output.data(), output.size());
    if (code == 2) throw std::runtime_error("planner input/output buffer error");
    auto result = Json::parse(output.data());
    if (code != 0 || !result.at("ok").get<bool>()) throw std::runtime_error(result.at("error").get<std::string>());
    return result; // 轨迹已检查维度、限位、最大步长和终点。
}

void boundedArm(const std::vector<double>& values, const Json& path) {
    if (values.size() != 7) throw std::runtime_error("right arm dimension changed");
    for (std::size_t i=0; i<values.size(); ++i)
        if (!std::isfinite(values[i]) || values[i] < path.at("lower").at(i).get<double>()-1e-6 ||
            values[i] > path.at("upper").at(i).get<double>()+1e-6)
            throw std::runtime_error("right arm position outside model limits");
}

double difference(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) throw std::runtime_error("position dimension changed");
    double result = 0;
    for (std::size_t i = 0; i < a.size(); ++i) result = std::max(result, std::abs(a[i] - b.at(i)));
    return result;
}

sdk::ControlGroupCommand positionCommand(const Positions& positions) {
    // 右臂轨迹帧和固定手部目标装进同一条命令，按同一时钟发布。
    sdk::ControlGroupCommand command;
    command.ttl_ms = 250;
    command.admission_deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(250);
    for (const auto& [name,values] : positions) {
        sdk::JointCommand joint;
        joint.group_name = name;
        joint.control_type = sdk::JointControlType::Joint;
        joint.motor_mode = sdk::MotorMode::Position;
        joint.position = values;
        joint.velocity.assign(values.size(),0.0); joint.effort.assign(values.size(),0.0);
        command.joint_cmd.push_back(std::move(joint));
    }
    return command;
}

class ThumbsUp {
public:
    ThumbsUp() { checked(handGoal, profile_); }
    ~ThumbsUp() { close(); }

    frs::TaskReport handle(const frs::Request& request) {
        try {
            // 主线程只校验和受理；保存完整 Request，让 worker 保留原 ID。
            const auto input = frs::json::parse(std::string(request.input()));
            if (!input.isObject() || !input.asObject().empty()) throw std::invalid_argument("run/stop accept no arguments");
            std::unique_lock lock(mutex_);
            if (request.command() == "stop") {
                if (!active_ || stop_) return frs::state().event(request, frs::Event::NotReady, "no cancellable task");
                stop_ = request; cancelled_ = true; changed_.notify_all();
                return frs::state().event(request, frs::Event::Accepted, "cancellation requested");
            }
            if (request.command() != "run") throw std::invalid_argument("unsupported command");
            if (active_ || blocked_ || closing_) return frs::state().event(request, frs::Event::Busy, "busy or requires restart");
            lock.unlock();
            if (worker_.joinable()) worker_.join();
            lock.lock();
            active_ = true; cancelled_ = false;
            frs::state().set(frs::ModuleState::Running, std::string(request.request_id()));
            try { worker_ = std::thread(&ThumbsUp::execute, this, request); }
            catch (...) { active_ = false; frs::state().set(frs::ModuleState::Idle); throw; }
            return frs::state().event(request, frs::Event::Accepted, "accepted");
        } catch (const std::exception& error) { return frs::state().event(request, frs::Event::Rejected, error.what()); }
    }

    void close() {
        // 先取消业务并唤醒等待，再 join；不能在 worker 结束前销毁 Runtime。
        { std::lock_guard lock(mutex_); closing_ = true; cancelled_ = true; changed_.notify_all(); }
        if (worker_.joinable()) worker_.join();
    }

private:
    bool waitUntil(Clock::time_point due) {
        std::unique_lock lock(mutex_);
        return !changed_.wait_until(lock, due, [&] { return cancelled_; });
    }

    void send(const frs::Request& request, frs::Event event, const std::string& message,
              double progress, const std::string& data = {}) {
        // event 构造，report 才发送；每个阶段都沿用原 Request 身份。
        auto report = frs::state().event(request, event, message, progress);
        if (!data.empty()) {
            report.content_type("application/json");
            report.data(std::vector<std::uint8_t>(data.begin(), data.end()));
        }
        frs::report(report);
    }

    void execute(frs::Request request) {
        bool leaseAttempted = false, published = false, cleanupFailed = false;
        std::string failure, data;
        Json path;
        try {
            // 1. 本例使用 FHM 接口映射；在申请资源前拒绝模式不匹配。
            const auto* mode = std::getenv("FRS_RUNTIME_MODE");
            if (!mode || std::string(mode) != "sim") throw std::runtime_error("this hand profile requires simulation");
            send(request, frs::Event::Running, "reading_feedback", 0);
            const auto initial = readFeedback(profile_);
            if (difference(initial.measured.at("right_manipulator"),initial.armCommand) > 0.15)
                throw std::runtime_error("initial arm measured/command positions disagree");
            // 2. Aurora 由 FRS 代理，宿主使用本 Atom ID 关联 owner。
            if (!frs::aurora().waitForEndpoints(std::chrono::milliseconds(500)))
                throw std::runtime_error("Aurora endpoints are not ready");
            // 台架默认的 PdStandHanging 不转发手部 DDS 用户命令。
            // 全局 FSM 由开发者先准备；本 Atom 申请右臂和右手，不替其他模块切模式。
            sdk::AuroraState state;
            const auto observed = frs::aurora().getAuroraState(state);
            if (!observed.success() || state.current_state.name != "UpperBodyUserCmd")
                throw std::runtime_error("prepare UpperBodyUserCmd in simulation before running this Atom");
            // 3. 与 FBA move_pose 一样：整机快照 -> MoveTools 右臂轨迹 -> 申请资源。
            send(request, frs::Event::Running, "planning", 0);
            path = planArm(initial.message);
            if (waitUntil(Clock::now())) {
                sdk::LeaseOption option; option.duration = std::chrono::milliseconds(8000);
                leaseAttempted = true;
                const auto lease = frs::aurora().registerLease(
                    {sdk::AuroraResource::RightManipulator,sdk::AuroraResource::RightEndEffector}, option);
                if (!lease.success() || !lease.usable()) throw std::runtime_error("arm/hand lease rejected: " + lease.operation.message);
                // 规划后再校验起点，不能把已经过时的轨迹拿来播放。
                const auto live = readFeedback(profile_);
                if (difference(live.armCommand,initial.armCommand)>0.15 ||
                    difference(live.measured.at("right_manipulator"),initial.armCommand)>0.15 ||
                    difference(live.measured.at(groupName),initial.measured.at(groupName))>0.15)
                    throw std::runtime_error("pose start changed while planning");
                const auto frames = path.at("positions").get<std::vector<std::vector<double>>>();
                auto previous = initial.armCommand;
                const auto begin = Clock::now();
                const auto timeout = std::chrono::duration<double>((frames.size()-1)/30.0+2.0);
                std::optional<Clock::time_point> stable;
                std::optional<std::uint64_t> lastSequence;
                bool reached = false;
                for (std::size_t index=0; Clock::now()-begin<timeout; ++index) {
                    const auto due = begin+period*index;
                    if (!waitUntil(due)) break;
                    if (Clock::now()>due+period*2) throw std::runtime_error("pose publication missed its deadline");
                    const auto feedback = readFeedback(profile_);
                    boundedArm(feedback.measured.at("right_manipulator"),path);
                    const auto& armFrame = frames.at(std::min(index,frames.size()-1));
                    if (difference(armFrame,previous)>0.15) throw std::runtime_error("arm frame step exceeds publication budget");
                    if (!waitUntil(Clock::now())) break;
                    // 4. 右臂消费轨迹帧，手部重复固定目标；先记尝试，再检查发布回执。
                    published = true;
                    const auto result = frs::aurora().publishControlGroupCommand(
                        positionCommand({{"right_manipulator",armFrame},{groupName,handGoal}}));
                    if (!result.success()) throw std::runtime_error("pose publication rejected: " + result.message);
                    previous = armFrame;
                    // 5. FBA 的条件：右臂轨迹已发布，手部实测在容差内稳定100 ms。
                    const double armError = difference(feedback.measured.at("right_manipulator"),armGoal);
                    const double error = difference(feedback.measured.at(groupName),handGoal);
                    const bool done = index+1>=frames.size();
                    const bool freshFrame = lastSequence!=feedback.sample.frs.sequence;
                    if (freshFrame) {
                        lastSequence = feedback.sample.frs.sequence;
                        if (error<=0.03) { if (!stable) stable=Clock::now(); }
                        else stable.reset();
                    }
                    const bool settled = stable && Clock::now()-*stable>=std::chrono::milliseconds(100);
                    if (!settled && Clock::now()-begin>=std::chrono::seconds(2))
                        throw std::runtime_error("thumbs-up hand feedback did not settle within 2 s");
                    data = Json{{"group",groupName},{"target",handGoal},{"measured",feedback.measured.at(groupName)},
                        {"arm_target",armGoal},{"arm_measured",feedback.measured.at("right_manipulator")},
                        {"arm_error_rad",armError},{"max_error_rad",error},{"sequence",std::to_string(feedback.sample.frs.sequence)}}.dump();
                    send(request,frs::Event::Running,done ? "settling" : "moving",
                         std::min(0.95,double(index+1)/frames.size()),data);
                    if (done && settled && freshFrame) { reached=true; break; }
                }
                if (!reached && waitUntil(Clock::now())) throw std::runtime_error("thumbs-up feedback did not settle after trajectory");
            }
        } catch (const std::exception& error) { failure = error.what(); }
        // 5. 正常、取消、异常共用收尾：停止目标帧，保持当前实测位置，然后释放。
        if (published) {
            try {
                const auto feedback = readFeedback(profile_);
                boundedArm(feedback.measured.at("right_manipulator"),path);
                const auto held = frs::aurora().publishControlGroupCommand(positionCommand(feedback.measured));
                if (!held.success()) throw std::runtime_error("measured hold rejected: " + held.message);
            } catch (const std::exception& error) { cleanupFailed = true; failure += "; hold: " + std::string(error.what()); }
        }
        if (leaseAttempted) {
            try {
                const auto released = frs::aurora().releaseLease();
                if (!released.success() || released.state != sdk::LeaseState::Unregistered)
                    throw std::runtime_error("arm/hand release was not confirmed");
            } catch (const std::exception& error) { cleanupFailed = true; failure += "; release: " + std::string(error.what()); }
        }
        std::lock_guard lock(mutex_);
        blocked_ = cleanupFailed;
        try {
            // 原任务终态和 stop 请求结果分开报告；释放未确认不能报成功。
            send(request, !failure.empty() ? frs::Event::Failed : cancelled_ ? frs::Event::Cancelled : frs::Event::Succeeded,
                 !failure.empty() ? failure : cancelled_ ? "pose held; resources released" : "trajectory published; hand targets reached; resources released", -1, data);
            if (stop_) send(*stop_, cleanupFailed ? frs::Event::Failed : frs::Event::Succeeded,
                            cleanupFailed ? failure : "pose held; resources released", -1);
        } catch (const std::exception& error) { blocked_ = true; std::cerr << "task report failed: " << error.what() << '\n'; }
        stop_.reset(); active_ = false;
        frs::state().set(blocked_ || closing_ ? frs::ModuleState::Stopped : frs::ModuleState::Idle);
    }

    Profile profile_;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::thread worker_;
    std::optional<frs::Request> stop_;
    bool active_{false}, cancelled_{false}, closing_{false}, blocked_{false};
};

int main() {
    std::unique_ptr<ThumbsUp> atom;
    try {
        atom = std::make_unique<ThumbsUp>();
        // 读取 atom.json，握手 ready，注册命令/关闭回调，再持续收请求。
        frs::start("atom.json");
        frs::on_command([&](const auto& request) {
            const auto receipt = atom->handle(request);
            if (receipt.result_code() != 0) frs::report(receipt); // 拒绝也持久化到原请求。
            return receipt;
        });
        frs::on_shutdown([&] { atom->close(); });
        frs::state().set(frs::ModuleState::Idle);
        frs::spin();
        frs::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; // stdout 由 Runtime 保持纯协议。
        try { frs::shutdown(); } catch (const std::exception& cleanup) { std::cerr << cleanup.what() << '\n'; }
        return 1;
    }
}
