#include <controller_module.hpp>
#include <time_helper.hpp>
#include <cmath>
#include <stdexcept>

class HoldController final : public frs::ControllerModule {
public:
    frs::RobotJointCommand prepare(frs::ControllerModuleContext& context,
                                   const std::string& command,
                                   const frs::json::Value& parameters) {
        if (command != "hold") throw std::runtime_error("unsupported command");
        const auto observation = context.state();
        // 关节数来自当前机器人；测量位置与模型 joint_names 使用相同顺序。
        count_ = context.limits.joint_count;
        if (!observation.fresh || observation.age < std::chrono::nanoseconds::zero() ||
            observation.age > std::chrono::milliseconds(100) || count_ == 0 ||
            !valid(observation.robot.joint_position))
            throw std::runtime_error("fresh joint state is required");
        // 保持当前测量姿态，避免第一帧直接跳到另一个目标。默认输出组为 whole_body、模式为 PVT。
        target_.position = observation.robot.joint_position;
        target_.velocity.assign(count_, 0.0);
        target_.effort.assign(count_, 0.0);
        // Controller 每帧必须提供完整 kp/kd；这里使用本次 hold 调用传入的逐关节增益。
        target_.kp = gains(parameters.at("kp"));
        target_.kd = gains(parameters.at("kd"));
        const auto* value = parameters.find("frames");
        const double frames = value ? value->asNumber() : 20;
        if (!std::isfinite(frames) || frames < 1 || frames > 1000 || std::floor(frames) != frames)
            throw std::runtime_error("invalid frames");
        frames_ = static_cast<std::size_t>(frames);
        return target_;
    }

    void idle(frs::ControllerModuleContext& context) override {
        count_ = context.limits.joint_count;
        if (target_.position.empty()) {
            target_.position = context.state().robot.joint_position;
            target_.velocity.assign(count_, 0); target_.effort.assign(count_, 0);
            target_.kp.assign(count_, 40); target_.kd.assign(count_, 3);
        }
        // 50 Hz 是本例的算法周期；清单省略的 frame_timeout_ms 使用框架默认 100 ms。
        frs::PeriodicTimer timer(50);
        while (!context.stopping()) {
            context.publish(target_);
            // 此保持示例仅用于仿真。实际算法应在满足姿态、支撑、速度条件时报告可交接。
            context.canSwitch(true);
            (void)timer.waitNext();
        }
    }

    frs::json::Value execute(frs::ControllerModuleContext& context,
                             const std::string& command, const frs::json::Value& parameters) override {
        (void)prepare(context, command, parameters);
        frs::PeriodicTimer timer(50.0);
        std::size_t sent = 0;
        while (sent < frames_ && !context.stopping()) {
            (void)timer.waitNext();
            if (context.stopping()) break;
            // 只提交这一帧；框架校验限位和步长，填入序号、时间与控制会话，再交给平台出口。
            context.publish(target_);
            ++sent;
            context.progress(static_cast<double>(sent) / frames_);
            timer.complete();
        }
        return frs::json::Value::Object{{"published_frames", static_cast<double>(sent)}};
    }

    std::vector<std::string> commands() const override { return {"hold"}; }

private:
    bool valid(const std::vector<double>& values) const {
        if (values.size() != count_) return false;
        for (const auto value : values) if (!std::isfinite(value)) return false;
        return true;
    }
    std::vector<double> gains(const frs::json::Value& value) const {
        std::vector<double> result;
        for (const auto& item : value.asArray()) result.push_back(item.asNumber());
        if (!valid(result)) throw std::runtime_error("invalid gain vector");
        for (const auto gain : result) if (gain < 0) throw std::runtime_error("negative gain");
        return result;
    }
    std::size_t count_{0}, frames_{20};
    frs::RobotJointCommand target_;
};

// 将控制器类与清单中的 ID 关联；加载入口和兼容性信息由框架生成。
FRS_REGISTER_CONTROLLER(HoldController, "tutorial_hold")
