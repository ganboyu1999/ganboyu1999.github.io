#pragma once
#include "detail/rigid_body_model.hpp"
#include <algorithm>
#include <limits>

namespace frs::base {
inline json::Value soleGeometry(const json::Value& model) {
    if (const auto* soles=model.find("sole_geometry")) return *soles;
    const auto& feet=model.at("foot_collision_geometry").asArray();
    if (feet.size()!=2) throw SdkError("BASE_MODEL_INVALID","two foot collision geometries are required");
    const auto vector=[](const json::Value& value) {
        const auto& a=value.asArray();
        if (a.size()!=3) throw SdkError("BASE_MODEL_INVALID","invalid collision vector");
        Vector result{a[0].asNumber(),a[1].asNumber(),a[2].asNumber()};
        for (double x:result) if (!std::isfinite(x)) throw SdkError("BASE_MODEL_INVALID","nonfinite collision geometry");
        return result;
    };
    json::Value::Array soles;
    for (const auto& foot:feet) {
        std::vector<Vector> points;
        for (const auto& shape:foot.asArray()) {
            const auto position=vector(shape.at("position"));
            const auto& q=shape.at("quaternion_wxyz").asArray();
            if (q.size()!=4) throw SdkError("BASE_MODEL_INVALID","invalid collision quaternion");
            double norm=0;
            for (const auto& x:q) norm+=x.asNumber()*x.asNumber();
            if (!std::isfinite(norm) || std::abs(norm-1)>1e-6) throw SdkError("BASE_MODEL_INVALID","invalid collision quaternion");
            const auto rotation=quaternion(q[0].asNumber(),q[1].asNumber(),q[2].asNumber(),q[3].asNumber());
            if (shape.at("type").asString()=="box") {
                const auto size=vector(shape.at("size"));
                for (double x:size) if (x<=0) throw SdkError("BASE_MODEL_INVALID","invalid collision size");
                if (std::abs(q[1].asNumber())+std::abs(q[2].asNumber())+std::abs(q[3].asNumber())>1e-6)
                    throw SdkError("BASE_MODEL_INVALID","axis-aligned sole box is required");
                for (int x:{-1,1}) for (int y:{-1,1})
                    points.push_back(add(position,{x*size[0]/2,y*size[1]/2,-size[2]/2}));
            } else if (shape.at("type").asString()=="cylinder") {
                const double radius=shape.at("radius").asNumber(), length=shape.at("length").asNumber();
                if (!std::isfinite(radius) || !std::isfinite(length) || radius<=0 || length<=0)
                    throw SdkError("BASE_MODEL_INVALID","invalid cylinder dimensions");
                const auto axis=rotate(rotation,{0,0,1});
                if (std::abs(std::abs(axis[0])-1)>1e-3 || std::abs(axis[1])+std::abs(axis[2])>1e-3)
                    throw SdkError("BASE_MODEL_INVALID","longitudinal horizontal sole cylinders are required");
                // Use bottom generators, not the cylinder's outer width, as support.
                for (int side:{-1,1}) points.push_back(add(position,
                    {side*axis[0]*length/2,side*axis[1]*length/2,side*axis[2]*length/2-radius}));
            } else throw SdkError("BASE_MODEL_INVALID","unsupported sole collision primitive");
        }
        if (points.size()<4) throw SdkError("BASE_MODEL_INVALID","sole contact region is degenerate");
        Vector lower=points.front(), upper=lower;
        for (const auto& point:points) for (int axis=0;axis<3;++axis) {
            lower[axis]=std::min(lower[axis],point[axis]); upper[axis]=std::max(upper[axis],point[axis]);
        }
        if (upper[2]-lower[2]>1e-3 || upper[0]-lower[0]<=0 || upper[1]-lower[1]<=0)
            throw SdkError("BASE_MODEL_INVALID","coplanar rectangular sole support is required");
        for (double x:{lower[0],upper[0]}) for (double y:{lower[1],upper[1]})
            if (std::none_of(points.begin(),points.end(),[&](Vector p){return std::abs(p[0]-x)<1e-6 && std::abs(p[1]-y)<1e-6;}))
                throw SdkError("BASE_MODEL_INVALID","sole support is not rectangular");
        soles.emplace_back(json::Value::Object{
            {"center",json::Value::Array{(lower[0]+upper[0])/2,(lower[1]+upper[1])/2,lower[2]}},
            {"half_size",json::Value::Array{(upper[0]-lower[0])/2,(upper[1]-lower[1])/2,0}}});
    }
    return soles;
}
}
