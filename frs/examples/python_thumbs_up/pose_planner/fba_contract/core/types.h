#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fba {
using Positions = std::map<std::string, std::vector<double>>;
struct MotionFrame { size_t repeat{}; Positions groups; };
struct RecordedMotion {
    enum class Kind { Sampled, Keyframes } kind{Kind::Sampled};
    std::string model;
    double period_s{};
    size_t sample_count{};
    std::map<std::string, std::vector<std::string>> groups;
    std::vector<MotionFrame> frames;
};
enum class CommandKind { PlayMotion, MoveArm, MovePose, ControlHand, ControlHead, ControlWaist, Move, Pause, Resume, Stop };
struct PoseTarget {
    enum class Execution { Trajectory, PositionTarget };
    CommandKind kind{CommandKind::MoveArm};
    std::string group;
    std::vector<double> position;
    double speed_ratio{0.2};
    double start_tolerance{};
    Execution execution{Execution::Trajectory};
};
struct Command {
    CommandKind kind{CommandKind::Stop};
    // move 质子保留 Aurora 原子命令名和已校验的参数 token。
    std::string name;
    std::vector<std::string> args;
    std::string group;
    std::vector<double> position;
    // Internal MoveTools joint-target request used for safe play_motion startup.
    std::vector<double> joint_target;
    std::array<double, 4> orientation{0, 0, 0, 1}; // xyzw, base-frame target
    double speed_ratio{0.2};
    std::vector<PoseTarget> targets;
    std::shared_ptr<const RecordedMotion> motion;
};
struct JointConfig {
    std::vector<double> lower, upper;
    double velocity{}, acceleration{}, jerk{}, max_step{}, tolerance{};
};
struct AuroraConfig {
    int discovery_timeout_ms{3000};
    uint32_t command_ttl_ms{100};
};
struct TopicConfig {
    std::string aurora_control_group_state;
    std::string aurora_estimator_state;
    std::string aurora_state;
    std::string robot_motion_state;
    static constexpr const char* simulation_pose_state = "/frs_fhm/estimator_state";
    static constexpr const char* map = "map";
    static constexpr const char* global_costmap = "global_costmap/costmap";
};
struct Config {
    TopicConfig topics;
    AuroraConfig aurora;
    double control_hz{30}, speed_ratio{0.2}, max_duration_s{60};
    double joint_tolerance{0.03}, start_tolerance{0.15}, max_joint_step{0.15};
    double start_pose_step_ratio{0.1};
    int state_age_ms{500}, state_wait_ms{3000}, settle_timeout_ms{2000};
    std::map<std::string, std::string> arm_groups;
    std::map<std::string, JointConfig> joint_groups;
    std::map<std::string, Command> gestures;
    std::string movetools_plugin;
    std::string planning_config_dir;
    std::string robot_config_root;
    std::string robot_model;
    std::map<std::string, std::vector<std::string>> playback_groups;
};
struct MotionResult {
    enum Code { Success, Cancelled, Failed } code{Success};
    std::string message;
};
class Cancellation {
public:
    enum class Phase { Running, PauseRequested, Paused, ResumeRequested };
    void cancel() { cancelled_.store(true); changed_.notify_all(); }
    bool cancelled() const { return cancelled_.load(); }
    bool requestPause() {
        auto expected=Phase::Running;
        return !cancelled() && phase_.compare_exchange_strong(expected,Phase::PauseRequested);
    }
    bool requestResume() {
        auto expected=Phase::Paused;
        return !cancelled() && phase_.compare_exchange_strong(expected,Phase::ResumeRequested);
    }
    Phase phase() const { return phase_.load(); }
    void paused() { phase_.store(Phase::Paused); }
    void resumed() { phase_.store(Phase::Running); }
    bool waitUntil(std::chrono::steady_clock::time_point time) const {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait_until(lock, time, [&] { return cancelled(); });
        return !cancelled();
    }
private:
    std::atomic<bool> cancelled_{false};
    std::atomic<Phase> phase_{Phase::Running};
    mutable std::mutex mutex_;
    mutable std::condition_variable changed_;
};
}
