#pragma once
#include "detail/rigid_body_model.hpp"
#include "controller_module.hpp"
#include "time_helper.hpp"

namespace frs {
// Model-based foot-pose IK, centroid feedback and quasi-static ground-force
// compensation. The controller owns trajectories, pacing and safe stop logic.
class BasePolicy final : public ControllerModule {
public:
    // A simulation may supply its own cycle clock. The default uses absolute
    // monotonic releases; overruns are counted and missed periods are skipped.
    using CycleClock = std::function<CycleTick(double)>;
    explicit BasePolicy(CycleClock clock = {}) : cycle_clock_(std::move(clock)) {}
    void initialize(ControllerModuleContext&, const json::Value&) override;
    void idle(ControllerModuleContext&) override;
    RobotJointCommand prepare(ControllerModuleContext&, const std::string&, const json::Value&);
    json::Value execute(ControllerModuleContext&, const std::string&, const json::Value&) override;
    bool supportsPause() const noexcept override { return true; }
    std::vector<std::string> commands() const override { return {"stand", "walk"}; }
private:
    json::Value configuration_{json::Value::Object{}};
    CycleClock cycle_clock_;
    RobotJointCommand prepared_;
    bool hanging_{false};
    base::RigidBodyModel gravity_model_;
};
} // namespace frs
