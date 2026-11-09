#pragma once

#include "controller_module.hpp"
#include "time_helper.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <mutex>
#include <thread>
#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace frs::base {
inline bool followJointTarget(RobotJointCommand& command,const RobotJointCommand& previous,
                              const FrameLimits& limits,double rate) {
    if (command.position.size()!=previous.position.size())
        throw SdkError("FRAME_SIZE_MISMATCH","planning command changed joint layout");
    if (command.position.size()!=limits.max_position_step.size() ||
        command.position.size()!=limits.velocity_abs_max.size())
        throw SdkError("FRAME_LIMITS_UNAVAILABLE","base output requires joint step and velocity limits");
    bool reached=true;
    for (std::size_t i=0;i<command.position.size();++i) {
        if (!std::isfinite(command.position[i]))
            throw SdkError("FRAME_NON_FINITE","nonfinite planning joint target");
        const double delta=command.position[i]-previous.position[i];
        const double step=std::min(limits.max_position_step[i],limits.velocity_abs_max[i]/rate);
        if (std::abs(delta)<=step) continue;
        command.position[i]=previous.position[i]+std::copysign(step,delta);
        if (std::abs(command.position[i]-previous.position[i])>step)
            command.position[i]=std::nextafter(command.position[i],previous.position[i]);
        reached=false;
    }
    return reached;
}
// The controller owns both rates. A bounded planning cycle supplies commands
// while the output cycle continues publishing the latest validated target.
inline json::Value streamCommands(ControllerModuleContext& context,
    const RobotJointCommand& first,double rate,
    const std::function<json::Value(ControllerModuleContext&)>& plan,
    const std::function<CycleTick(double)>& clock = {}) {
    using Clock=std::chrono::steady_clock;
    std::mutex mutex;
    auto latest=first,previous_command=first;
    auto updated=Clock::now();
    bool done=false;
    std::exception_ptr error;
    json::Value result;
    std::atomic<bool> abort{false};
    PeriodicTimer timer(rate);
    auto input=context;
    input.publish=[&](const RobotJointCommand& command) {
        if (abort.load()) throw SdkError("BASE_OUTPUT_CANCELLED","output cycle stopped");
        std::lock_guard lock(mutex); latest=command; updated=Clock::now();
    };
    input.state=[&] {
        if (abort.load()) throw SdkError("BASE_OUTPUT_CANCELLED","output cycle stopped");
        return context.state();
    };
    input.get=[&](const std::string& name,std::chrono::milliseconds timeout) {
        if (abort.load()) throw SdkError("BASE_OUTPUT_CANCELLED","output cycle stopped");
        return context.get(name,timeout);
    };
    std::thread planner([&] {
        try {
#if defined(__linux__)
            // A planner inherits the publisher's FIFO policy unless explicitly
            // reset. It must remain preemptible by the command output thread.
            sched_param scheduling{};
            if (::pthread_setschedparam(::pthread_self(),SCHED_OTHER,&scheduling)!=0)
                throw SdkError("BASE_SCHEDULING_FAILED","cannot make the planner preemptible");
#endif
            auto registration=input.monitorCurrentThread ? input.monitorCurrentThread() : nullptr;
            auto value=plan(input);
            std::lock_guard lock(mutex); result=std::move(value); done=true;
        } catch (...) {
            std::lock_guard lock(mutex); error=std::current_exception(); done=true;
        }
    });
    std::size_t frames=0,overruns=0;
    double max_age=0,max_interval=0;
    auto started=Clock::now(),previous=started;
    try {
        for (;;) {
            const auto tick=clock ? clock(rate) : timer.waitNext();
            RobotJointCommand command;
            bool finished;
            double age;
            {
                std::lock_guard lock(mutex);
                if (error) std::rethrow_exception(error);
                command=latest; finished=done;
                age=std::chrono::duration<double>(Clock::now()-updated).count();
            }
            if (age>std::chrono::duration<double>(context.frame_timeout).count())
                throw SdkError("BASE_COMMAND_STALE","planning command exceeded controller frame_timeout_ms");
            // Sample the joint trajectory at the output rate using the model's
            // velocity and step limits, including after a delayed planning tick.
            const bool reached=followJointTarget(command,previous_command,context.limits,rate);
            context.publish(command);
            previous_command=command;
            if (tick.missed_release || (!clock && !timer.withinDeadline())) ++overruns;
            const auto now=Clock::now();
            if (frames) max_interval=std::max(max_interval,std::chrono::duration<double>(now-previous).count());
            max_age=std::max(max_age,age); previous=now; ++frames;
            timer.complete();
            if (finished && reached) break;
        }
    } catch (...) {
        abort.store(true); planner.join(); throw;
    }
    planner.join();
    const double elapsed=std::chrono::duration<double>(previous-started).count();
    result.asObject()["output_frames"]=static_cast<double>(frames);
    result.asObject()["output_overrun_count"]=static_cast<double>(overruns);
    result.asObject()["output_rate_hz"]=elapsed>0 && frames>1 ? (frames-1)/elapsed : 0;
    result.asObject()["max_output_interval_ms"]=1000*max_interval;
    result.asObject()["max_command_age_ms"]=1000*max_age;
    return result;
}
} // namespace frs::base
