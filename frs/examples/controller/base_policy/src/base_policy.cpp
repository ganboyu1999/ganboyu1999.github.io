#include "base_policy.hpp"
#include "time_helper.hpp"
#include "detail/quasi_static_gait.hpp"
#include "detail/command_stream.hpp"
#include "detail/sole_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>

namespace frs {
namespace {
using Value = json::Value;
double number(const Value& p, const char* key, double fallback, double lo, double hi) {
    const auto* v = p.find(key); const double n = v ? v->asNumber() : fallback;
    if (!std::isfinite(n) || n < lo || n > hi)
        throw SdkError("BASE_PARAMETER_INVALID", std::string("invalid ") + key, "base_policy");
    return n;
}
double smooth(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return u*u*u*(10 + u*(-15 + 6*u));
}
std::vector<double> gains(const Value& parameters, const char* array_key, const char* scalar_key,
                          std::size_t count, double fallback, double low, double high) {
    const auto* value = parameters.find(array_key);
    if (!value) return std::vector<double>(count, number(parameters, scalar_key, fallback, low, high));
    if (!value->isArray() || value->asArray().size() != count)
        throw SdkError("BASE_PARAMETER_INVALID", std::string("invalid ") + array_key);
    std::vector<double> result;
    for (const auto& item : value->asArray()) {
        const double n = item.asNumber();
        if (!std::isfinite(n) || n < low || n > high)
            throw SdkError("BASE_PARAMETER_INVALID", std::string("invalid ") + array_key);
        result.push_back(n);
    }
    return result;
}
struct Leg {
    std::array<std::size_t, 5> index{};
    std::array<double, 5> direction{};
};
Leg leg(const Value& v, std::size_t count, std::set<std::size_t>& used) {
    static const std::array<const char*, 5> names{
        "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"};
    Leg l;
    for (std::size_t i = 0; i < names.size(); ++i) {
        const auto& item = v.at(names[i]);
        const double n = item.at("index").asNumber();
        const double direction = item.at("direction").asNumber();
        if (!std::isfinite(n) || n < 0 || n >= count || std::floor(n) != n ||
            (direction != -1 && direction != 1) || !used.insert(static_cast<std::size_t>(n)).second)
            throw SdkError("BASE_JOINT_MAP_INVALID", "joint indexes must be unique; directions must be +/-1", "base_policy");
        l.index[i] = static_cast<std::size_t>(n); l.direction[i] = direction;
    }
    return l;
}
std::array<double, 3> inverseLeg(double x, double z, double thigh, double shin) {
    const double d2 = x*x + z*z;
    const double c = (d2 - thigh*thigh - shin*shin) / (2*thigh*shin);
    if (c <= -1 || c >= 1)
        throw SdkError("BASE_IK_UNREACHABLE", "leg target is outside the configured workspace", "base_policy");
    const double knee = std::acos(c);
    const double hip = std::atan2(-x, z) - std::atan2(shin*std::sin(knee), thigh + shin*std::cos(knee));
    return {hip, knee, -hip-knee};
}
std::array<double, 2> foot(double phase, double step, double height) {
    phase -= std::floor(phase);
    if (phase >= 0.15 && phase < 0.35) {
        const double u = (phase - 0.15) / 0.2;
        const double bump = 16*u*u*(1-u)*(1-u);
        return {step*(smooth(u)-0.5), height*bump};
    }
    const double u = (phase >= 0.35 ? phase - 0.35 : phase + 0.65) / 0.8;
    return {step*(0.5-smooth(u)), 0};
}
double supportShift(double phase) {
    phase-=std::floor(phase);
    if (phase<.15) return smooth(phase/.15);
    if (phase<.35) return 1;
    if (phase<.65) return 1-2*smooth((phase-.35)/.3);
    if (phase<.85) return -1;
    return -1+smooth((phase-.85)/.15);
}
bool swinging(double phase) {
    phase-=std::floor(phase);
    return (phase>=.15 && phase<.35) || (phase>=.65 && phase<.85);
}
}
void BasePolicy::initialize(ControllerModuleContext& context, const Value& configuration) {
    if (!configuration.isObject()) throw SdkError("BASE_PARAMETER_INVALID", "configuration must be an object");
    configuration_ = configuration;
    gravity_model_ = {};
    prepared_ = {};
    if (!configuration_.find("left_leg") && context.get) {
        const auto model = context.get("controller/model", std::chrono::milliseconds(100));
        const auto& names = model.at("joint_names").asArray();
        gravity_model_.load(model.at("rigid_body_model"), names.size());
        for (const std::string side : {"left", "right"}) {
            Value::Object mapping;
            Value::Array chain;
            for (const char* role : {"hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll"}) {
                const auto name = side + "_" + role + "_joint";
                const auto found = std::find_if(names.begin(), names.end(), [&](const Value& item) { return item.asString() == name; });
                if (found == names.end()) throw SdkError("BASE_MODEL_REQUIRED", "model has no joint: " + name);
                chain.emplace_back(static_cast<double>(found-names.begin()));
            }
            configuration_.asObject()[side + "_kinematic_chain"] = chain;
            for (const char* role : {"hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"}) {
                const auto name = side + "_" + role + "_joint";
                const auto found = std::find_if(names.begin(), names.end(), [&](const Value& item) { return item.asString() == name; });
                if (found == names.end()) throw SdkError("BASE_MODEL_REQUIRED", "model has no joint: " + name);
                const auto index = static_cast<std::size_t>(found-names.begin());
                const auto& axis = model.at("rigid_body_model").at("joints").asArray()[index].at("axis").asArray();
                const auto expected = std::string(role).find("roll") != std::string::npos ? 0 : 1;
                for (int i=0;i<3;++i)
                    if (std::abs(std::abs(axis[i].asNumber())-(i==expected ? 1 : 0))>1e-6)
                        throw SdkError("BASE_JOINT_MAP_INVALID", "planar gait joint axis does not match role: " + name);
                mapping[role] = Value::Object{{"index", static_cast<double>(index)}, {"direction", axis[expected].asNumber()}};
            }
            configuration_.asObject()[side + "_leg"] = mapping;
        }
        const auto& geometry = model.at("leg_geometry").at("left");
        const double thigh = geometry.at("thigh_m").asNumber();
        const double shin = geometry.at("shin_m").asNumber();
        configuration_.asObject()["thigh_m"] = thigh;
        configuration_.asObject()["shin_m"] = shin;
        configuration_.asObject()["standing_height_m"] = thigh + shin - .015;
        configuration_.asObject()["gravity_m_s2"] = model.at("gravity_m_s2");
        configuration_.asObject()["sole_geometry"] = base::soleGeometry(model);


    }
}
void BasePolicy::idle(ControllerModuleContext& context) {
    auto command = prepare(context, "stand", Value::Object{});
    const double rate = number(configuration_, "rate_hz", 500, 20, 500);
    PeriodicTimer timer(rate);
    while (!context.stopping()) {
        const auto state = context.state();
        if (gravity_model_.available()) {
            const auto effort = gravity_model_.gravityEffort(state.robot.joint_position,
                state.robot.base_quaternion_xyzw, .5, !hanging_);
            for (std::size_t i = 0; i < effort.size(); ++i)
                command.effort[i] = std::clamp(effort[i], -context.limits.effort_abs_max[i], context.limits.effort_abs_max[i]);
        }
        context.publish(command);
        const auto& r = state.robot;
        const auto& q = r.base_quaternion_xyzw;
        const double roll = std::atan2(2*(q[3]*q[0]+q[1]*q[2]), 1-2*(q[0]*q[0]+q[1]*q[1]));
        const double pitch = std::asin(std::clamp(2*(q[3]*q[1]-q[2]*q[0]), -1.0, 1.0));
        const auto below = [](const std::vector<double>& values, double maximum) {
            return std::all_of(values.begin(), values.end(), [maximum](double v) { return std::abs(v) < maximum; });
        };
        const bool supported = hanging_ || r.contact_state == std::vector<std::uint8_t>{1,1};
        context.canSwitch(supported && std::abs(roll) < .15 && std::abs(pitch) < .15 &&
            below(r.joint_velocity, .15) && below(r.base_linear_velocity, .05) && below(r.base_angular_velocity, .1));
        if (cycle_clock_) (void)cycle_clock_(rate); else (void)timer.waitNext();
    }
}
RobotJointCommand BasePolicy::prepare(ControllerModuleContext& context, const std::string& command, const Value& input) {
    if (command != "stand" && command != "walk")
        throw SdkError("CONTROLLER_COMMAND_UNSUPPORTED", "base_policy supports stand and walk", "base_policy");
    if (!input.isObject()) throw SdkError("BASE_PARAMETER_INVALID", "command parameters must be an object");
    Value p = configuration_;
    if (gravity_model_.available()) {
        const auto model = context.get("controller/model", std::chrono::milliseconds(100));
        for (const char* key : {"kp_joint", "kd_joint"}) {
            p.asObject()[key] = model.at(key);
            for (auto& value : p.asObject()[key].asArray())
                if (value.asNumber() == 0) value = key == std::string("kp_joint") ? 40.0 : 3.0;
            if (p.at(key).asArray().size()!=context.limits.joint_count)
                throw SdkError("BASE_MODEL_INVALID", "standing gain layout differs from joint model");
        }
        for (const std::string side : {"left", "right"}) {
            const auto& chain=p.at(side+"_kinematic_chain").asArray();
            const auto hip=static_cast<std::size_t>(chain[1].asNumber());
            const auto ankle=static_cast<std::size_t>(chain[5].asNumber());
            p.asObject()["kp_joint"].asArray()[ankle]=p.at("kp_joint").asArray()[hip];
        }
    }
    for (const auto& item : input.asObject()) p.asObject()[item.first] = item.second;
    for (const char* key : {"left_leg", "right_leg", "thigh_m", "shin_m", "standing_height_m"})
        if (!p.find(key)) throw SdkError("BASE_MODEL_REQUIRED", std::string("base_policy requires ") + key, "base_policy");
    const auto state = context.state();
    hanging_ = state.robot.contact_state != std::vector<std::uint8_t>{1, 1};
    if (command == "walk" && (hanging_ || state.robot.contact_state != std::vector<std::uint8_t>{1, 1}))
        throw SdkError("BASE_GROUND_SUPPORT_REQUIRED", "walk requires grounded Aurora standing", "base_policy");
    std::set<std::size_t> used;
    (void)leg(p.at("left_leg"), context.limits.joint_count, used);
    (void)leg(p.at("right_leg"), context.limits.joint_count, used);
    const double thigh = number(p, "thigh_m", 0, 0.05, 1);
    const double shin = number(p, "shin_m", 0, 0.05, 1);
    const double height = number(p, "standing_height_m", 0, 0.1, thigh+shin-0.005);
    (void)inverseLeg(0, height, thigh, shin);
    (void)number(p, "rate_hz", 500, 20, 500);
    (void)number(p, "balance_kp", 0.3, 0, 1);
    (void)number(p, "balance_kd", 0.05, 0, 0.2);
    (void)number(p, "duration_s", 5, 0.1, 30);
    const double cycle = number(p, "cycle_s", gravity_model_.available() ? .9 : 2, gravity_model_.available() ? .9 : 1.5, gravity_model_.available() ? 6 : 10);
    if (!gravity_model_.available() && (p.find("velocity_m_s") || p.find("direction")))
        throw SdkError("BASE_PARAMETER_INVALID", "velocity_m_s and direction require a rigid-body model");
    const double step=gravity_model_.available() ? base::walkingStep(p,cycle).forward : number(p,"step_length_m",.02,0,.05);
    if (gravity_model_.available()) (void)number(p, "crouch_height_m", .025, 0, .08);
    if (p.find("step_width_m") && !gravity_model_.available())
        throw SdkError("BASE_PARAMETER_INVALID", "step_width_m requires a rigid-body model");
    if (gravity_model_.available()) (void)number(p, "step_width_m", .28, .2, .4);
    const double lift = number(p, "step_height_m", gravity_model_.available() ? .025 : .01, 0, gravity_model_.available() ? .04 : .025);
    (void)number(p, "lateral_shift_m", 0.01, 0, 0.14);
    (void)inverseLeg(step/2, height, thigh, shin);
    (void)inverseLeg(0, height-lift, thigh, shin);
    RobotJointCommand first;
    first.position = state.robot.joint_position;
    const auto count=first.position.size();
    if (context.limits.position_lower.size()!=count || context.limits.position_upper.size()!=count ||
        context.limits.max_position_step.size()!=count)
        throw SdkError("BASE_MODEL_INVALID", "complete initial hold position limits are required", "base_policy");
    for (std::size_t i=0;i<count;++i) {
        const double lower=context.limits.position_lower[i],upper=context.limits.position_upper[i];
        const double step_limit=context.limits.max_position_step[i];
        if (!std::isfinite(first.position[i]) || !std::isfinite(lower) || !std::isfinite(upper) ||
            lower>upper || !std::isfinite(step_limit) || step_limit<=0)
            throw SdkError("BASE_STATE_INVALID", "invalid initial hold position or limits", "base_policy");
        const double target=std::clamp(first.position[i],lower,upper);
        if (std::abs(target-first.position[i])>step_limit)
            throw SdkError("POSITION_STEP_EXCEEDED", "base initial hold exceeds the position step limit", "base_policy");
        first.position[i]=target;
    }
    first.velocity.assign(first.position.size(), 0);
    first.effort = state.robot.joint_effort;
    if (gravity_model_.available()) {
        const auto effort=gravity_model_.gravityEffort(state.robot.joint_position,state.robot.base_quaternion_xyzw,.5,!hanging_);
        for (std::size_t i=0;i<effort.size();++i)
            first.effort[i]=std::clamp(effort[i],-context.limits.effort_abs_max[i],context.limits.effort_abs_max[i]);
    }
    first.kp = gains(p, "kp_joint", "kp", first.position.size(), 40, 1, 2000);
    first.kd = gains(p, "kd_joint", "kd", first.position.size(), 3, .1, 100);
    prepared_ = first;
    return first;
}
Value BasePolicy::execute(ControllerModuleContext& context, const std::string& command, const Value& input) {
    (void)prepare(context, command, input);
    const auto fresh = context.readyForIdle;
    context.readyForIdle = [state = context.state, fresh, grounded = !hanging_] {
        if (!fresh()) return false;
        const auto& r = state().robot;
        const auto& q = r.base_quaternion_xyzw;
        const double roll = std::atan2(2*(q[3]*q[0]+q[1]*q[2]), 1-2*(q[0]*q[0]+q[1]*q[1]));
        const double pitch = std::asin(std::clamp(2*(q[3]*q[1]-q[2]*q[0]), -1.0, 1.0));
        const auto below = [](const std::vector<double>& values, double maximum) {
            return std::all_of(values.begin(), values.end(), [maximum](double value) { return std::abs(value) < maximum; });
        };
        return (!grounded || r.contact_state == std::vector<std::uint8_t>{1,1}) &&
            std::abs(roll) < .15 && std::abs(pitch) < .15 && below(r.joint_velocity,.15) &&
            below(r.base_linear_velocity,.05) && below(r.base_angular_velocity,.1);
    };

    if (command != "stand" && command != "walk")
        throw SdkError("CONTROLLER_COMMAND_UNSUPPORTED", "base_policy supports stand and walk", "base_policy");
    if (!input.isObject()) throw SdkError("BASE_PARAMETER_INVALID", "command parameters must be an object");
    Value p = configuration_;
    for (const auto& item : input.asObject()) p.asObject()[item.first] = item.second;
    const auto initial = context.state();
    if (hanging_) {
        const double rate = number(p, "rate_hz", 500, 20, 500);
        const double duration = number(p, "duration_s", 5, .1, 30);
        PeriodicTimer timer(rate);
        std::size_t frames = 0, overruns = 0;
        double elapsed = 0, active_time = 0;
        bool paused = false;
        for (;;) {
            const auto tick = cycle_clock_ ? cycle_clock_(rate) : timer.waitNext();
            const auto compute_started = std::chrono::steady_clock::now();
            const auto state = context.state();
            auto out = prepared_;
            if (gravity_model_.available()) {
                const auto effort = gravity_model_.gravityEffort(state.robot.joint_position,
                    state.robot.base_quaternion_xyzw, .5, false);
                const double blend = smooth(elapsed / .5);
                for (std::size_t i = 0; i < effort.size(); ++i)
                    out.effort[i] += blend * (effort[i] - out.effort[i]);
            }
            if (context.reportComputeTime)
                context.reportComputeTime(std::chrono::steady_clock::now() - compute_started);
            context.publish(out);
            if (tick.missed_release || (!cycle_clock_ && !timer.withinDeadline())) ++overruns;
            ++frames; elapsed += 1 / rate;
            if (paused != context.pausing()) {
                paused = context.pausing(); context.reportPaused(paused);
            }
            if (!paused) active_time += 1 / rate;
            context.progress(std::min(active_time / duration, 1.0));
            timer.complete();
            if ((context.stopping() || active_time >= duration) && context.readyForIdle()) break;
        }
        return Value::Object{{"algorithm", "hanging_pd_hold"}, {"frames", static_cast<double>(frames)},
            {"overrun_count",static_cast<double>(overruns)}};
    }
    if (gravity_model_.available()) {
        const double output_rate=number(p,"rate_hz",500,20,500);
        p.asObject()["rate_hz"]=std::min(output_rate,100.0);
        const auto plan=[&](ControllerModuleContext& input) {
            return base::ruleBasedGait(input,gravity_model_,prepared_,p,command=="walk",cycle_clock_);
        };
        if (!cycle_clock_ && output_rate>100) return base::streamCommands(context,prepared_,output_rate,plan);
        return plan(context);
    }
    for (const char* key : {"left_leg", "right_leg", "thigh_m", "shin_m", "standing_height_m"})
        if (!p.find(key)) throw SdkError("BASE_MODEL_REQUIRED", std::string("base_policy requires ") + key, "base_policy");
    std::set<std::size_t> used;
    const auto left = leg(p.at("left_leg"), context.limits.joint_count, used);
    const auto right = leg(p.at("right_leg"), context.limits.joint_count, used);
    const double thigh = number(p, "thigh_m", 0, 0.05, 1);
    const double shin = number(p, "shin_m", 0, 0.05, 1);
    const double height = number(p, "standing_height_m", 0, 0.1, thigh+shin-0.005);
    const double rate = number(p, "rate_hz", 500, 20, 500);
    const auto kp = gains(p, "kp_joint", "kp", initial.robot.joint_position.size(), 40, 1, 2000);
    const auto kd = gains(p, "kd_joint", "kd", initial.robot.joint_position.size(), 3, .1, 100);
    const double balance_kp = number(p, "balance_kp", 0.3, 0, 1);
    const double balance_kd = number(p, "balance_kd", 0.05, 0, 0.2);
    const double duration = number(p, "duration_s", 5, 0.1, 30);
    const double cycle = number(p, "cycle_s", 2, 1.5, 10);
    const double step = number(p, "step_length_m", 0.02, 0, 0.05);
    const double lift = number(p, "step_height_m", 0.01, 0, 0.025);
    const double shift = number(p, "lateral_shift_m", 0.01, 0, 0.14);
    const auto neutral = inverseLeg(0, height, thigh, shin);
    const auto standing = prepared_.position.empty() ? initial.robot.joint_position : prepared_.position;
    auto previous = standing;
    bool was_paused = false;
    double gait_time = 0;
    double active_time = 0;
    double magnitude = 0, ramp_from = 0, ramp_target = 0, ramp_start = 0, ramp_time = 0;
    PeriodicTimer timer(rate);
    std::size_t frames = 0, overruns = 0;
    for (;;) {
        const auto tick = cycle_clock_ ? cycle_clock_(rate) : timer.waitNext();
        const auto compute_started = std::chrono::steady_clock::now();
        const auto state = context.state();
        const double elapsed = active_time;
        const bool finishing = context.stopping() || elapsed >= duration;
        double target = command == "walk" && !finishing && !context.pausing() ? 1 : 0;
        if (target==0 && magnitude==1 && swinging(gait_time/cycle)) target=1;
        if (target != ramp_target) {
            ramp_from = magnitude; ramp_target = target; ramp_start = ramp_time;
        }
        // Fade both swing and stance trajectories to zero before a paused or
        // idle entry. Keep feedback and joint publication running throughout.
        magnitude = ramp_from + (ramp_target-ramp_from)*smooth(ramp_time-ramp_start);
        const double amplitude = magnitude;
        const double phase = gait_time/cycle;
        const auto lfoot = foot(phase, step*amplitude, lift*amplitude);
        const auto rfoot = foot(phase+0.5, step*amplitude, lift*amplitude);
        const auto l = inverseLeg(lfoot[0], height-lfoot[1], thigh, shin);
        const auto r = inverseLeg(rfoot[0], height-rfoot[1], thigh, shin);
        const auto& q = state.robot.base_quaternion_xyzw;
        const double roll = std::atan2(2*(q[3]*q[0]+q[1]*q[2]), 1-2*(q[0]*q[0]+q[1]*q[1]));
        const double pitch = std::asin(std::clamp(2*(q[3]*q[1]-q[2]*q[0]), -1.0, 1.0));
        const double roll_feedback = balance_kp*roll + balance_kd*state.robot.base_angular_velocity[0];
        const double pitch_feedback = balance_kp*pitch + balance_kd*state.robot.base_angular_velocity[1];
        const double support_shift = amplitude*supportShift(phase);
        const double weight_shift = std::atan2(shift*support_shift, height);
        RobotJointCommand out;
        out.position = standing;
        out.effort = prepared_.effort.empty() ? initial.robot.joint_effort : prepared_.effort;
        out.kp = kp; out.kd = kd;
        const auto apply = [&](const Leg& map, const std::array<double, 3>& angles) {
            out.position[map.index[0]] += map.direction[0]*weight_shift;
            for (std::size_t i = 0; i < 3; ++i)
                out.position[map.index[i+1]] += map.direction[i+1]*(angles[i]-neutral[i]);
            out.position[map.index[3]] += map.direction[3]*pitch_feedback;
            out.position[map.index[4]] += map.direction[4]*(roll_feedback-weight_shift);
        };
        apply(left, l); apply(right, r);
        if (context.limits.effort_abs_max.size()==out.effort.size())
            for (std::size_t i=0;i<out.effort.size();++i)
                out.effort[i]=std::clamp(out.effort[i],-context.limits.effort_abs_max[i],context.limits.effort_abs_max[i]);
        out.velocity.resize(standing.size());
        for (std::size_t i = 0; i < standing.size(); ++i) out.velocity[i] = (out.position[i]-previous[i])*rate;
        if (context.reportComputeTime)
            context.reportComputeTime(std::chrono::steady_clock::now() - compute_started);
        context.publish(out);
        if (tick.missed_release || (!cycle_clock_ && !timer.withinDeadline())) ++overruns;
        previous = out.position;
        ++frames;
        context.progress(std::min(elapsed/duration, 1.0));
        const bool settled = ramp_target == 0 && magnitude == 0 && context.readyForIdle();
        if (settled && finishing) break;
        if (settled && context.pausing()) {
            if (!was_paused) { context.reportPaused(true); was_paused = true; }
        } else if (was_paused && !context.pausing()) {
            context.reportPaused(false); was_paused = false;
        }
        if (!was_paused && magnitude==1 && ramp_target==1) {
            double next = gait_time+1/rate;
            gait_time=next;
        }
        ramp_time += 1/rate;
        if (!context.pausing() && !context.stopping()) active_time += 1/rate;
        timer.complete();
    }
    return Value::Object{{"frames", static_cast<double>(frames)}, {"command", command},
        {"overrun_count",static_cast<double>(overruns)}};
}
} // namespace frs
