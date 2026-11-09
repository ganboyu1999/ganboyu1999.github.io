#pragma once

#include "controller_module.hpp"
#include "detail/rigid_body_model.hpp"
#include "detail/footstep_feedback.hpp"
#include <algorithm>
#include <set>

namespace frs::base {
inline double quintic(double u) {
    u=std::clamp(u,0.0,1.0); return u*u*u*(10+u*(-15+6*u));
}
struct WalkingStep { double forward, lateral; };
inline WalkingStep walkingStep(const json::Value& parameters,double cycle) {
    const auto* direction=parameters.find("direction");
    const std::string name=direction ? direction->asString() : "forward";
    if (name!="forward" && name!="backward" && name!="left" && name!="right")
        throw SdkError("BASE_PARAMETER_INVALID","direction must be forward, backward, left or right");
    const bool sideways=name=="left" || name=="right";
    if (parameters.find("velocity_m_s") && parameters.find("step_length_m"))
        throw SdkError("BASE_PARAMETER_INVALID","velocity_m_s and step_length_m are mutually exclusive");
    const double speed_limit=sideways ? .04 : .1;
    const auto* velocity=parameters.find("velocity_m_s");
    const auto* length=parameters.find("step_length_m");
    const double value=velocity ? velocity->asNumber() : length ? length->asNumber() : sideways ? .018 : .035;
    const double step=velocity ? value*cycle/2 : value;
    if (!std::isfinite(value) || (direction && value<0) || std::abs(step)>.08+1e-12 ||
        2*std::abs(step)>speed_limit*cycle+1e-12)
        throw SdkError("BASE_PARAMETER_INVALID","direction, speed and cycle exceed the walking workspace");
    const double signed_step=std::clamp(step,-.08,.08)*(name=="backward" || name=="right" ? -1 : 1);
    return sideways ? WalkingStep{0,signed_step} : WalkingStep{signed_step,0};
}
inline Vector rotationError(const Rotation& current,const Rotation& desired) {
    Vector result{};
    for (int column=0;column<3;++column) {
        const auto c=Vector{current[0][column],current[1][column],current[2][column]};
        const auto d=Vector{desired[0][column],desired[1][column],desired[2][column]};
        const auto error=cross(c,d);
        for (int i=0;i<3;++i) result[i]+=.5*error[i];
    }
    return result;
}
inline std::array<double,6> solve(std::array<std::array<double,7>,6> augmented) {
    for (int column=0;column<6;++column) {
        int pivot=column;
        for (int row=column+1;row<6;++row)
            if (std::abs(augmented[row][column])>std::abs(augmented[pivot][column])) pivot=row;
        std::swap(augmented[column],augmented[pivot]);
        if (std::abs(augmented[column][column])<1e-12) throw SdkError("BASE_IK_SINGULAR","leg Jacobian is singular");
        const double divisor=augmented[column][column];
        for (int j=column;j<7;++j) augmented[column][j]/=divisor;
        for (int row=0;row<6;++row) if (row!=column) {
            const double factor=augmented[row][column];
            for (int j=column;j<7;++j) augmented[row][j]-=factor*augmented[column][j];
        }
    }
    std::array<double,6> result{};
    for (int i=0;i<6;++i) result[i]=augmented[i][6];
    return result;
}
inline void inverseFoot(const RigidBodyModel& model,std::vector<double>& joints,
                        const std::vector<double>& quaternion,const std::array<std::size_t,6>& chain,
                        int side,Vector desired_position,const Rotation& desired_rotation,const FrameLimits& limits) {
    for (int iteration=0;iteration<12;++iteration) {
        const auto current=model.evaluate(joints,quaternion,.5,false);
        const auto linear=subtract(desired_position,current.foot_position[side]);
        const auto angular=rotationError(current.foot_rotation[side],desired_rotation);
        if (dot(linear,linear)<1e-10 && dot(angular,angular)<1e-8) return;
        std::array<double,6> error{linear[0],linear[1],linear[2],.2*angular[0],.2*angular[1],.2*angular[2]};
        std::array<std::array<double,6>,6> jacobian{};
        for (int column=0;column<6;++column) {
            const auto joint=chain[column];
            const auto rotation=current.joint_axis[joint];
            const auto translation=cross(rotation,subtract(current.foot_position[side],current.joint_anchor[joint]));
            for (int row=0;row<3;++row) {
                jacobian[row][column]=translation[row];
                jacobian[row+3][column]=.2*rotation[row];
            }
        }
        std::array<std::array<double,7>,6> normal{};
        for (int i=0;i<6;++i) {
            for (int k=0;k<6;++k) {
                normal[i][6]+=jacobian[k][i]*error[k];
                for (int j=0;j<6;++j) normal[i][j]+=jacobian[k][i]*jacobian[k][j];
            }
            normal[i][i]+=1e-7;
        }
        const auto increment=solve(normal);
        for (int i=0;i<6;++i) {
            const auto index=chain[i];
            joints[index]=std::clamp(joints[index]+std::clamp(increment[i],-.08,.08),
                                     limits.position_lower[index],limits.position_upper[index]);
        }
    }
    const auto pose=model.evaluate(joints,quaternion,.5,false);
    const auto linear=subtract(desired_position,pose.foot_position[side]);
    const auto angular=rotationError(pose.foot_rotation[side],desired_rotation);
    if (dot(linear,linear)>4e-8 || dot(angular,angular)>4e-6)
        throw SdkError("BASE_IK_UNREACHABLE","foot pose is outside the configured leg workspace");
}
inline Vector readVector(const json::Value& value) {
    if (!value.isArray() || value.asArray().size()!=3) throw SdkError("BASE_STATE_INVALID","expected three-vector");
    Vector result{};
    for (int i=0;i<3;++i) {
        result[i]=value.asArray()[i].asNumber();
        if (!std::isfinite(result[i])) throw SdkError("BASE_STATE_INVALID","nonfinite physical input");
    }
    return result;
}
inline json::Value physicalState(ControllerModuleContext& context) {
    const auto state=context.get("controller/state",std::chrono::milliseconds(10));
    const auto stamp=std::stoll(state.at("source_timestamp_ns").asString());
    const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (stamp<=0 || stamp>now || now-stamp>100000000) throw SdkError("CONTROLLER_STATE_STALE","physical state is stale");
    return state;
}
inline std::vector<double> orientationXyzw(const json::Value& state) {
    const auto& value=state.at("quaternion_wxyz");
    if (!value.isArray() || value.asArray().size()!=4)
        throw SdkError("BASE_STATE_INVALID","expected quaternion wxyz");
    const auto& q=value.asArray();
    double norm=0;
    for (const auto& item:q) {
        const double component=item.asNumber();
        if (!std::isfinite(component)) throw SdkError("BASE_STATE_INVALID","nonfinite physical quaternion");
        norm+=component*component;
    }
    if (std::abs(norm-1)>1e-3) throw SdkError("BASE_STATE_INVALID","physical quaternion is not normalized");
    return {q[1].asNumber(),q[2].asNumber(),q[3].asNumber(),q[0].asNumber()};
}
inline std::vector<double> jointPositions(const json::Value& state,std::size_t count) {
    const auto& values=state.at("joint_position");
    if (!values.isArray() || values.asArray().size()!=count)
        throw SdkError("BASE_STATE_INVALID","physical joint layout differs from model");
    std::vector<double> result;
    for (const auto& value:values.asArray()) {
        const double position=value.asNumber();
        if (!std::isfinite(position)) throw SdkError("BASE_STATE_INVALID","nonfinite physical joint position");
        result.push_back(position);
    }
    return result;
}
class ObservationClock {
public:
    explicit ObservationClock(const json::Value& state): previous_(timestamp(state)) {}
    double advance(const json::Value& state) {
        const auto current=timestamp(state);
        if (current<previous_ || current-previous_>100'000'000ULL)
            throw SdkError("BASE_STATE_INVALID","control observation clock jumped");
        const double elapsed=static_cast<double>(current-previous_)*1e-9;
        previous_=current;
        return elapsed;
    }
private:
    static std::uint64_t timestamp(const json::Value& state) {
        const auto* value=state.find("control_timestamp_ns");
        if (!value || !value->isString())
            throw SdkError("BASE_STATE_INVALID","control observation clock is missing");
        const auto& text=value->asString();
        if (text.empty() || text.size()>20 || !std::all_of(text.begin(),text.end(),[](char c) { return c>='0' && c<='9'; }))
            throw SdkError("BASE_STATE_INVALID","invalid control observation clock");
        try { return std::stoull(text); }
        catch (const std::exception&) { throw SdkError("BASE_STATE_INVALID","control observation clock overflow"); }
    }
    std::uint64_t previous_;
};
inline json::Value ruleBasedGait(ControllerModuleContext& context,const RigidBodyModel& model,
    const RobotJointCommand& first,const json::Value& parameters,bool walking,
    const std::function<CycleTick(double)>& clock) {
    const auto parameter=[&](const char* key,double fallback,double lower,double upper) {
        const auto* value=parameters.find(key); const double result=value ? value->asNumber() : fallback;
        if (!std::isfinite(result) || result<lower || result>upper) throw SdkError("BASE_PARAMETER_INVALID",key);
        return result;
    };
    const double rate=parameter("rate_hz",100,20,500), duration=parameter("duration_s",walking ? 24 : 5,.1,30);
    const double cycle=parameter("cycle_s",.9,.9,6), lift=parameter("step_height_m",.025,0,.04);
    const auto step=walkingStep(parameters,cycle);
    const double crouch_depth=parameter("crouch_height_m",.025,0,.08);
    const double step_width=parameter("step_width_m",.28,.2,.4);
    std::array<std::array<std::size_t,6>,2> chains{};
    std::set<std::size_t> used;
    for (int side=0;side<2;++side) {
        const auto& indexes=parameters.at(side==0 ? "left_kinematic_chain" : "right_kinematic_chain").asArray();
        if (indexes.size()!=6) throw SdkError("BASE_MODEL_INVALID","six leg hinges are required");
        for (int i=0;i<6;++i) {
            const double value=indexes[i].asNumber();
            if (!std::isfinite(value) || value<0 || value>=context.limits.joint_count || std::floor(value)!=value ||
                !used.insert(static_cast<std::size_t>(value)).second)
                throw SdkError("BASE_JOINT_MAP_INVALID","leg chains must use distinct valid joint indexes");
            chains[side][i]=static_cast<std::size_t>(value);
        }
    }
    const auto initial=physicalState(context);
    ObservationClock observation_clock(initial);
    const auto base_position=readVector(initial.at("position"));
    const auto captured_orientation=orientationXyzw(initial);
    const auto& q=captured_orientation;
    const double yaw=std::atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));
    const std::vector<double> orientation{0,0,std::sin(yaw/2),std::cos(yaw/2)};
    const Vector forward{std::cos(yaw),std::sin(yaw),0}, lateral{-std::sin(yaw),std::cos(yaw),0};
    double orientation_sign=0;
    for (int i=0;i<4;++i) orientation_sign+=captured_orientation[i]*orientation[i];
    orientation_sign=orientation_sign<0 ? -1 : 1;
    const auto reference=model.evaluate(first.position,captured_orientation,.5,false);
    std::array<Vector,2> feet{};
    for (int side=0;side<2;++side) feet[side]=add(base_position,reference.foot_position[side]);
    auto anchors=feet;
    const auto nominal=reference.support_error;
    const auto& soles=parameters.at("sole_geometry").asArray();
    if (soles.size()!=2) throw SdkError("BASE_MODEL_INVALID","two rectangular soles are required");
    const double inward_margin=.6*std::min(readVector(soles[0].at("half_size"))[1],readVector(soles[1].at("half_size"))[1]);
    const double gravity=parameters.at("gravity_m_s2").asNumber();
    if (!std::isfinite(gravity) || gravity<=0 || nominal[2]-crouch_depth<=.1)
        throw SdkError("BASE_MODEL_INVALID","positive pendulum height and gravity are required");
    const double omega=std::sqrt(gravity/(nominal[2]-crouch_depth));
    FootstepFeedback footstep_feedback(cycle,omega,forward,lateral,step_width);
    const auto nominal_feet=[&](double seconds) {
        std::array<Vector,2> result{};
        const double gait=std::max(0.0,seconds), phase=gait/cycle-std::floor(gait/cycle);
        const int lap=static_cast<int>(std::floor(gait/cycle));
        for (int side=0;side<2;++side) {
            Vector midpoint{};
            for (int i=0;i<3;++i) midpoint[i]=.5*(anchors[0][i]+anchors[1][i]);
            const double offset=dot(subtract(midpoint,anchors[side]),forward);
            const double start=lap==0 ? 0 : side==0 ? 2*lap-1 : 2*lap;
            const double end=side==0 ? 2*lap+1 : 2*lap+2;
            const double begin=side==0 ? .15 : .65, finish=side==0 ? .45 : .95;
            const double u=std::clamp((phase-begin)/(finish-begin),0.0,1.0);
            result[side]=anchors[side];
            const double distance=start+(end-start)*quintic(u);
            for (int i=0;i<2;++i) result[side][i]+=forward[i]*(step.forward*distance+offset*(lap==0 ? quintic(u) : 1))+
                lateral[i]*step.lateral*distance;
            result[side][2]+=lift*16*u*u*(1-u)*(1-u);
        }
        return result;
    };
    const auto planned_feet=[&](double seconds) {
        auto result=nominal_feet(seconds);
        for (int side=0;side<2;++side) result[side]=add(result[side],footstep_feedback.offset(side,seconds));
        return result;
    };
    const auto support_load=[&](double seconds) {
        if (seconds<0) return .5;
        const double phase=seconds/cycle-std::floor(seconds/cycle);
        if (phase<.15) return .5*(1-quintic(phase/.15));
        if (phase<.45) return 0.0;
        if (phase<.65) return quintic((phase-.45)/.2);
        if (phase<.95) return 1.0;
        return 1-.5*quintic((phase-.95)/.05);
    };
    // Bounded LIPM solution: preview the support pressure center on both sides
    // of the current instant, rather than moving the CoM onto each stance foot.
    constexpr double preview_spacing=.02;
    std::array<double,201> preview_weights{};
    double preview_total=0;
    for (int sample=-100;sample<=100;++sample) {
        const double weight=.5*omega*std::exp(-omega*std::abs(sample*preview_spacing))*preview_spacing*(std::abs(sample)==100 ? .5 : 1);
        preview_weights[sample+100]=weight; preview_total+=weight;
    }
    const auto preview=[&](double seconds) {
        Vector result{};
        for (int sample=-100;sample<=100;++sample) {
            const double future=seconds+sample*preview_spacing;
            const auto planned=planned_feet(future);
            const double fraction=support_load(future);
            const double weight=preview_weights[sample+100];
            for (int i=0;i<2;++i)
                result[i]+=weight*(fraction*planned[0][i]+(1-fraction)*planned[1][i]+nominal[i]-(2*fraction-1)*inward_margin*lateral[i]);
        }
        for (int i=0;i<2;++i) result[i]/=preview_total;
        return result;
    };
    auto previous=first.position, target=first.position;
    auto previous_com=add(base_position,reference.center_of_mass);
    auto previous_desired=previous_com;
    Vector velocity{};
    double active_time=0,gait_time=-2,time=0,stop_start=-1,stop_load=.5,resume_start=-1,unsupported_time=0;
    Vector stop_com{}, resume_com{};
    bool paused=false, gait_advancing=walking;
    double crouch=0,crouch_from=0,crouch_target=0,crouch_start=0;
    PeriodicTimer timer(rate);
    std::size_t frames=0,overruns=0;
    RobotJointCommand last_output=first;
    for (;;) {
        const auto tick=clock ? clock(rate) : timer.waitNext();
        const auto compute_started=std::chrono::steady_clock::now();
        (void)context.state();
        const auto physical=physicalState(context);
        const double dt=observation_clock.advance(physical);
        if (frames && dt==0) {
            if (context.reportComputeTime)
                context.reportComputeTime(std::chrono::steady_clock::now()-compute_started);
            context.publish(last_output);
            if (tick.missed_release || (!clock && !timer.withinDeadline())) ++overruns;
            timer.complete();
            continue;
        }
        // Plan at this observation's timestamp so trajectory differences and
        // feedback use the same interval when state sampling is irregular.
        time+=dt;
        if (gait_advancing) gait_time+=dt;
        if (!context.pausing() && !context.stopping()) active_time+=dt;
        // Root pose and joints must come from the same adapter snapshot.
        const auto measured=jointPositions(physical,first.position.size());
        const auto& forces=physical.at("contact_force").asArray();
        if (forces.size()!=2 || !std::isfinite(forces[0].asNumber()) || !std::isfinite(forces[1].asNumber()) ||
            forces[0].asNumber()<0 || forces[1].asNumber()<0)
            throw SdkError("BASE_SUPPORT_INVALID","finite ground support forces are required");
        if (forces[0].asNumber()+forces[1].asNumber()<1) unsupported_time+=dt;
        else unsupported_time=0;
        if (unsupported_time>.05) throw SdkError("BASE_SUPPORT_LOST","both feet lost ground support");
        const auto root=readVector(physical.at("position"));
        const auto actual_orientation=orientationXyzw(physical);
        const auto actual=model.evaluate(measured,actual_orientation,.5,false);
        const auto com=add(root,actual.center_of_mass);
        if (frames && dt>0) for (int i=0;i<3;++i)
            velocity[i]+=(1-std::exp(-dt/.045))*((com[i]-previous_com[i])/dt-velocity[i]);
        auto controlled_orientation=captured_orientation;
        double orientation_norm=0;
        for (int i=0;i<4;++i) {
            controlled_orientation[i]+=(orientation_sign*orientation[i]-captured_orientation[i])*quintic(time);
            orientation_norm+=controlled_orientation[i]*controlled_orientation[i];
        }
        for (auto& value:controlled_orientation) value/=std::sqrt(orientation_norm);
        const auto desired_rotation=base::quaternion(controlled_orientation[3],controlled_orientation[0],controlled_orientation[1],controlled_orientation[2]);
        const double phase=std::max(0.0,gait_time)/cycle-std::floor(std::max(0.0,gait_time)/cycle);
        const bool left_swing=walking && gait_time>=0 && phase>=.15 && phase<.45;
        const bool right_swing=walking && gait_time>=0 && phase>=.65 && phase<.95;
        const bool finishing=context.stopping() || active_time>=duration;
        const bool stopping=(finishing || context.pausing()) && !left_swing && !right_swing;
        if (walking && !stopping && !paused) {
            const auto reference=preview(gait_time), before=preview(gait_time-.01), after=preview(gait_time+.01);
            Vector reference_velocity{};
            for (int i=0;i<2;++i) reference_velocity[i]=(after[i]-before[i])/.02;
            const double liftoff=(std::floor(gait_time/cycle)+(phase<.5 ? .15 : .65))*cycle;
            const auto intended=nominal_feet(liftoff);
            const double width_delta=dot(subtract(subtract(intended[0],intended[1]),subtract(anchors[0],anchors[1])),lateral);
            footstep_feedback.update(gait_time,dt,2*step.forward/cycle,com,velocity,reference,reference_velocity,
                dot(subtract(actual.foot_position[0],actual.foot_position[1]),lateral),width_delta);
        }
        double load=.5;
        if (walking) {
            load=support_load(gait_time);
            feet=planned_feet(gait_time);
        }
        if (paused) load=.5;
        if (resume_start>=0) {
            load=.5+(load-.5)*quintic((time-resume_start)/2);
            if (time-resume_start>=2) resume_start=-1;
        }
        if (stopping) {
            if (stop_start<0) { stop_start=time; stop_load=load; stop_com=previous_desired; resume_start=-1; }
            load=stop_load+(.5-stop_load)*quintic(time-stop_start);
        } else if (stop_start>=0 && !paused) stop_start=-1;
        const double crouch_request=walking && !stopping && !paused ? 1 : 0;
        if (crouch_request!=crouch_target) { crouch_from=crouch; crouch_target=crouch_request; crouch_start=time; }
        crouch=crouch_from+(crouch_target-crouch_from)*quintic((time-crouch_start)/2);
        const auto desired_at=[&](double offset) {
            const double instant=time+offset;
            const double gait=gait_time+(walking && !stopping && !paused ? offset : 0);
            const auto planned=walking ? planned_feet(gait) : feet;
            double fraction=walking ? support_load(gait) : .5;
            if (paused) fraction=.5;
            if (resume_start>=0) fraction=.5+(fraction-.5)*quintic((instant-resume_start)/2);
            if (stopping) fraction=stop_load+(.5-stop_load)*quintic(instant-stop_start);
            Vector result{};
            for (int i=0;i<3;++i) result[i]=fraction*planned[0][i]+(1-fraction)*planned[1][i]+nominal[i];
            if (walking) {
                const auto previewed=stopping || paused ? Vector{} : preview(gait);
                for (int i=0;i<2;++i) {
                    if (stopping || paused) result[i]=stop_com[i]+quintic((instant-stop_start)/2)*(result[i]-stop_com[i]);
                    else if (resume_start>=0) result[i]=resume_com[i]+quintic((instant-resume_start)/2)*(previewed[i]-resume_com[i]);
                    else result[i]=previewed[i];
                }
            }
            const double lowered=crouch_from+(crouch_target-crouch_from)*quintic((instant-crouch_start)/2);
            result[2]-=crouch_depth*lowered;
            return result;
        };
        const auto desired=desired_at(0);
        // Fixed trajectory spacing avoids differentiating scheduling jitter.
        constexpr double spacing=.01;
        const auto before=desired_at(-spacing),after=desired_at(spacing);
        Vector desired_velocity{},desired_acceleration{};
        for (int i=0;i<3;++i) {
            desired_velocity[i]=(after[i]-before[i])/(2*spacing);
            desired_acceleration[i]=(after[i]-2*desired[i]+before[i])/(spacing*spacing);
        }
        Vector acceleration{};
        for (int i=0;i<3;++i) {
            acceleration[i]=std::clamp(desired_acceleration[i]+40*std::clamp(desired[i]-com[i],-.03,.03)+12*(desired_velocity[i]-velocity[i]),-3.0,3.0);
        }
        const double height=nominal[2]-crouch_depth*crouch;
        Vector cop=com;
        for (int i=0;i<2;++i) cop[i]-=height*acceleration[i]/(gravity+acceleration[2]);
        const auto attitude_error=rotationError(base::quaternion(actual_orientation[3],actual_orientation[0],actual_orientation[1],actual_orientation[2]),desired_rotation);
        const auto& spatial_velocity=physical.at("velocity").asArray();
        if (spatial_velocity.size()!=6) throw SdkError("BASE_STATE_INVALID","expected six-dimensional base velocity");
        for (const auto& value:spatial_velocity)
            if (!std::isfinite(value.asNumber())) throw SdkError("BASE_STATE_INVALID","nonfinite base velocity");
        cop[0]-=(8*attitude_error[1]-1.6*spatial_velocity[1].asNumber())/(gravity+acceleration[2]);
        cop[1]+=(8*attitude_error[0]-1.6*spatial_velocity[0].asNumber())/(gravity+acceleration[2]);
        double force_load=load;
        // An early touchdown is already a usable support contact. Do not keep
        // assigning all reaction force to the scheduled stance leg.
        const double contact_threshold=.03*(forces[0].asNumber()+forces[1].asNumber());
        if ((!left_swing && !right_swing) || (forces[0].asNumber()>contact_threshold && forces[1].asNumber()>contact_threshold)) {
            const double separation=dot(subtract(actual.foot_position[0],actual.foot_position[1]),lateral);
            if (separation<=.05) throw SdkError("BASE_SUPPORT_INVALID","feet must have separated support regions");
            force_load=std::clamp(dot(subtract(subtract(cop,root),actual.foot_position[1]),lateral)/separation,0.0,1.0);
        }
        Vector cop_offset{};
        for (int i=0;i<2;++i) cop_offset[i]=cop[i]-root[i]-force_load*actual.foot_position[0][i]-(1-force_load)*actual.foot_position[1][i];
        const auto& sole=parameters.at("sole_geometry").asArray()[force_load>=.5 ? 0 : 1];
        const auto center=readVector(sole.at("center")),half=readVector(sole.at("half_size"));
        const double cop_forward=std::clamp(dot(cop_offset,forward),center[0]-.8*half[0],center[0]+.8*half[0]);
        const double cop_lateral=std::clamp(dot(cop_offset,lateral),center[1]-.8*half[1],center[1]+.8*half[1]);
        for (int i=0;i<2;++i) cop_offset[i]=forward[i]*cop_forward+lateral[i]*cop_lateral;
        cop_offset[2]=center[2]-half[2];
        for (int pass=0;pass<3;++pass) {
            const auto candidate=model.evaluate(target,controlled_orientation,.5,false);
            const auto desired_base=subtract(desired,candidate.center_of_mass);
            for (int side=0;side<2;++side)
                inverseFoot(model,target,controlled_orientation,chains[side],side,subtract(feet[side],desired_base),reference.foot_rotation[side],context.limits);
        }
        RobotJointCommand output=first;
        output.position=target;
        output.velocity.resize(target.size());
        const auto statics=model.evaluate(measured,actual_orientation,force_load,true,acceleration,cop_offset);
        for (std::size_t i=0;i<target.size();++i) {
            output.velocity[i]=dt>0 ? (target[i]-previous[i])/dt : 0;
            output.effort[i]=std::clamp(first.effort[i]+quintic(time)*(statics.effort[i]-first.effort[i]),
                                      -context.limits.effort_abs_max[i],context.limits.effort_abs_max[i]);
        }
        if (context.reportComputeTime)
            context.reportComputeTime(std::chrono::steady_clock::now()-compute_started);
        context.publish(output);
        if (tick.missed_release || (!clock && !timer.withinDeadline())) ++overruns;
        last_output=output;
        context.progress(std::min(active_time/duration,1.0));
        ++frames;
        if (stopping && time-stop_start>=2 && context.readyForIdle()) {
            if (finishing) break;
            if (!paused) { paused=true; context.reportPaused(true); }
        }
        if (paused && !context.pausing()) {
            paused=false; stop_start=-1; anchors=feet; gait_time=-2;
            footstep_feedback.reset();
            resume_com=desired; resume_start=time; context.reportPaused(false);
        }
        gait_advancing=walking && !stopping && !paused;
        previous=target; previous_com=com; previous_desired=desired;
        timer.complete();
    }
    return json::Value::Object{{"frames",static_cast<double>(frames)},{"command",walking ? "walk" : "stand"},
        {"overrun_count",static_cast<double>(overruns)},
        {"max_footstep_adjustment_m",footstep_feedback.maxAdjustment()}};
}
} // namespace frs::base
