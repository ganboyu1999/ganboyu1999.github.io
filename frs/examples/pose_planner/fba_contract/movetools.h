#pragma once
#include <memory>
#include <utility>
#include "motion/trajectory.h"

namespace fba {
// 插件只收取快照和目标、返回轨迹；不得读取 FRS、发布控制或保留调用方引用。
// ABI v3 adds the optional keyframe compiler to the planner contract. Rebuild old plugins.
// C++17 DTO requires a compatible compiler and standard library ABI.
inline constexpr unsigned MOVETOOLS_PLUGIN_ABI = 3;
class MotionPlanner {
public:
    virtual ~MotionPlanner() = default;
    virtual JointTrajectory plan(const Command&, const RobotSnapshot&) = 0;
    // Compile sparse fviz keyframes into a fixed-period executable motion.
    // Plugins that do not provide this capability keep the legacy direct replay path.
    virtual bool compileMotion(const RecordedMotion&, const RobotSnapshot&,
                               RecordedMotion&, std::string&) { return false; }
};
using PlannerVersion = unsigned (*)();
using PlannerCreate = MotionPlanner* (*)(const Config*);
using PlannerDestroy = void (*)(MotionPlanner*);

class MoveTools {
public:
    explicit MoveTools(std::shared_ptr<const Config> config);
    ~MoveTools();
    MoveTools(const MoveTools&) = delete;
    MoveTools& operator=(const MoveTools&) = delete;
    JointTrajectory plan(const Command&, const RobotSnapshot&);
    bool compileMotion(const RecordedMotion&, const RobotSnapshot&, RecordedMotion&, std::string&);
private:
    void load();
    std::shared_ptr<const Config> config_;
    void* library_{};
    MotionPlanner* planner_{};
    PlannerDestroy destroy_{};
};
}
