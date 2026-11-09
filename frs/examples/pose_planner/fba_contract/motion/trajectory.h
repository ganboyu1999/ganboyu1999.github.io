#pragma once
#include <Eigen/Geometry>
#include "core/types.h"

namespace fba {
struct JointGroupState { std::vector<double> position, velocity, effort; };
struct RobotSnapshot {
    std::map<std::string, JointGroupState> measured, command;
    Eigen::Affine3d base{Eigen::Affine3d::Identity()};
    uint64_t sequence{};
};
struct JointTrajectory {
    std::string group;
    double dt{};
    std::vector<std::vector<double>> positions;
    std::vector<double> lower, upper;
};
}
