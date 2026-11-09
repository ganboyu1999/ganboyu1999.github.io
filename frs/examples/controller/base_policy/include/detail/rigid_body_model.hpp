#pragma once

#include "json.hpp"
#include "types.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace frs::base {
using Vector = std::array<double, 3>;
using Rotation = std::array<Vector, 3>;
inline Vector add(Vector a, Vector b) { for (int i=0;i<3;++i) a[i]+=b[i]; return a; }
inline Vector subtract(Vector a, Vector b) { for (int i=0;i<3;++i) a[i]-=b[i]; return a; }
inline Vector cross(Vector a, Vector b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline double dot(Vector a, Vector b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline Vector rotate(const Rotation& r, Vector v) { return {dot(r[0],v),dot(r[1],v),dot(r[2],v)}; }
inline Rotation multiply(const Rotation& a,const Rotation& b) {
    Rotation r{};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) for (int k=0;k<3;++k) r[i][j]+=a[i][k]*b[k][j];
    return r;
}
inline Rotation quaternion(double w,double x,double y,double z) {
    return {{{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)},
             {2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)},
             {2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}}};
}
inline Rotation axisRotation(Vector a,double angle) {
    const double s=std::sin(angle/2),c=std::cos(angle/2);
    return quaternion(c,a[0]*s,a[1]*s,a[2]*s);
}

// Recursive rigid-body statics: compensate link gravity and subtract the
// planned ground-reaction forces through each supporting foot's Jacobian.
class RigidBodyModel {
    struct Body { int parent; double mass; Vector position,center; Rotation rotation; };
    struct Joint { int body; Vector position,axis; double reference; };
    std::vector<Body> bodies_;
    std::vector<Joint> joints_;
    std::vector<int> body_joint_;
    std::array<int,2> feet_{};
    int base_{0};
    double gravity_{0},mass_{0};
    static Vector vector(const json::Value& value) {
        if (!value.isArray() || value.asArray().size()!=3) throw SdkError("BASE_MODEL_INVALID","expected three-vector");
        Vector result{};
        for (int i=0;i<3;++i) {
            result[i]=value.asArray()[i].asNumber();
            if (!std::isfinite(result[i])) throw SdkError("BASE_MODEL_INVALID","nonfinite model vector");
        }
        return result;
    }
    static int index(const json::Value& v,int count) {
        const auto n=v.asNumber();
        if (!std::isfinite(n) || n<0 || n>=count || std::floor(n)!=n) throw SdkError("BASE_MODEL_INVALID","invalid body index");
        return static_cast<int>(n);
    }
    bool descendant(int body,int ancestor) const {
        for (;;) {
            if (body==ancestor) return true;
            if (body==0) return false;
            body=bodies_[body].parent;
        }
    }
public:
    struct Statics {
        std::vector<double> effort;
        Vector support_error, center_of_mass;
        std::array<Vector,2> foot_position;
        std::array<Rotation,2> foot_rotation;
        std::vector<Vector> joint_anchor, joint_axis;
    };
    bool available() const noexcept { return !bodies_.empty(); }
    void load(const json::Value& model,std::size_t joint_count) {
        bodies_.clear(); joints_.clear(); body_joint_.clear(); mass_=0;
        const auto& bodies=model.at("bodies").asArray();
        if (bodies.empty() || bodies.size()>1024) throw SdkError("BASE_MODEL_INVALID","invalid model size");
        base_=index(model.at("base_body"),static_cast<int>(bodies.size()));
        if (base_==0) throw SdkError("BASE_MODEL_INVALID","floating base must be a robot body");
        gravity_=model.at("gravity_m_s2").asNumber();
        if (!std::isfinite(gravity_) || gravity_<=0) throw SdkError("BASE_MODEL_INVALID","invalid gravity");
        for (std::size_t i=0;i<bodies.size();++i) {
            const auto& b=bodies[i];
            const auto& q=b.at("quaternion_wxyz").asArray();
            if (q.size()!=4) throw SdkError("BASE_MODEL_INVALID","invalid model quaternion");
            double norm=0; for (const auto& v:q) norm+=v.asNumber()*v.asNumber();
            if (!std::isfinite(norm) || std::abs(norm-1)>1e-6) throw SdkError("BASE_MODEL_INVALID","invalid model quaternion");
            const double mass=b.at("mass_kg").asNumber();
            if (!std::isfinite(mass) || mass<0) throw SdkError("BASE_MODEL_INVALID","invalid link mass");
            bodies_.push_back({index(b.at("parent"),static_cast<int>(i ? i : 1)),mass,vector(b.at("position")),
                vector(b.at("center")),quaternion(q[0].asNumber(),q[1].asNumber(),q[2].asNumber(),q[3].asNumber())});
        }
        for (std::size_t i=0;i<bodies_.size();++i) if (descendant(static_cast<int>(i),base_)) mass_+=bodies_[i].mass;
        body_joint_.assign(bodies_.size(),-1);
        for (const auto& j:model.at("joints").asArray()) {
            const auto axis=vector(j.at("axis"));
            const double reference=j.at("reference").asNumber();
            const int body=index(j.at("body"),static_cast<int>(bodies_.size()));
            if (std::abs(dot(axis,axis)-1)>1e-6 || !std::isfinite(reference) || !descendant(body,base_))
                throw SdkError("BASE_MODEL_INVALID","invalid joint axis or reference");
            if (body_joint_[body]>=0) throw SdkError("BASE_MODEL_INVALID","one hinge per body is required");
            body_joint_[body]=static_cast<int>(joints_.size());
            joints_.push_back({body,vector(j.at("position")),axis,reference});
        }
        if (joints_.size()!=joint_count || mass_<=0) throw SdkError("BASE_MODEL_INVALID","invalid model joint layout");
        const auto& feet=model.at("foot_bodies").asArray();
        if (feet.size()!=2) throw SdkError("BASE_MODEL_INVALID","two feet are required");
        for (int i=0;i<2;++i) {
            feet_[i]=index(feet[i],static_cast<int>(bodies_.size()));
            if (!descendant(feet_[i],base_)) throw SdkError("BASE_MODEL_INVALID","foot is outside robot model");
        }
        if (feet_[0]==feet_[1]) throw SdkError("BASE_MODEL_INVALID","feet must be distinct");
    }
    Statics evaluate(const std::vector<double>& q,const std::vector<double>& xyzw,double left_support,
                     bool compute_effort = true, Vector acceleration = {}, Vector cop_offset = {},
                     bool ground_support = true) const {
        if (q.size()!=joints_.size() || xyzw.size()!=4) throw SdkError("BASE_MODEL_INVALID","state does not match model");
        if (!available() || !std::isfinite(left_support) || left_support<0 || left_support>1)
            throw SdkError("BASE_MODEL_INVALID","invalid support fraction");
        for (double value:q) if (!std::isfinite(value)) throw SdkError("BASE_MODEL_INVALID","nonfinite joint position");
        double norm=0; for (double value:xyzw) norm+=value*value;
        if (!std::isfinite(norm) || std::abs(norm-1)>1e-3) throw SdkError("BASE_MODEL_INVALID","invalid base quaternion");
        std::vector<Vector> position(bodies_.size()),center(bodies_.size()),anchor(joints_.size()),axis(joints_.size());
        std::vector<Rotation> rotation(bodies_.size());
        rotation[0]=quaternion(1,0,0,0);
        for (std::size_t i=1;i<bodies_.size();++i) {
            const auto& b=bodies_[i];
            position[i]=add(position[b.parent],rotate(rotation[b.parent],b.position));
            rotation[i]=multiply(rotation[b.parent],b.rotation);
            if (static_cast<int>(i)==base_) {
                position[i]={0,0,0}; rotation[i]=quaternion(xyzw[3],xyzw[0],xyzw[1],xyzw[2]);
            }
            if (body_joint_[i]>=0) {
                const auto j=static_cast<std::size_t>(body_joint_[i]);
                const auto& joint=joints_[j];
                anchor[j]=add(position[i],rotate(rotation[i],joint.position));
                axis[j]=rotate(rotation[i],joint.axis);
                rotation[i]=multiply(rotation[i],axisRotation(joint.axis,q[j]-joint.reference));
                position[i]=subtract(anchor[j],rotate(rotation[i],joint.position));
            }
            center[i]=add(position[i],rotate(rotation[i],b.center));
        }
        std::vector<double> effort(joints_.size(),0);
        for (std::size_t j=0;compute_effort && j<joints_.size();++j) {
            for (std::size_t b=1;b<bodies_.size();++b) if (descendant(static_cast<int>(b),joints_[j].body))
                effort[j]+=dot(axis[j],cross(subtract(center[b],anchor[j]),
                    {bodies_[b].mass*acceleration[0],bodies_[b].mass*acceleration[1],bodies_[b].mass*(gravity_+acceleration[2])}));
            for (int f=0;ground_support && f<2;++f) if (descendant(feet_[f],joints_[j].body)) {
                const double load=mass_*(f==0 ? left_support : 1-left_support);
                const auto force=Vector{load*acceleration[0],load*acceleration[1],load*(gravity_+acceleration[2])};
                effort[j]-=dot(axis[j],cross(add(subtract(position[feet_[f]],anchor[j]),cop_offset),force));
            }
        }
        Vector com{};
        for (std::size_t b=1;b<bodies_.size();++b) if (descendant(static_cast<int>(b),base_))
            for (int k=0;k<3;++k) com[k]+=center[b][k]*bodies_[b].mass/mass_;
        Vector support{};
        for (int k=0;k<3;++k) support[k]=left_support*position[feet_[0]][k]+(1-left_support)*position[feet_[1]][k];
        return {effort,subtract(com,support),com,
                {position[feet_[0]],position[feet_[1]]},{rotation[feet_[0]],rotation[feet_[1]]},
                std::move(anchor),std::move(axis)};
    }
    std::vector<double> gravityEffort(const std::vector<double>& q,const std::vector<double>& xyzw,double left_support,
                                      bool ground_support = true) const {
        return evaluate(q,xyzw,left_support,true,{},{},ground_support).effort;
    }
};
} // namespace frs::base
