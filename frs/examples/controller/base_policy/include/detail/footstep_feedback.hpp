#pragma once

#include "detail/rigid_body_model.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace frs::base {
// Capture point xi = c + v/omega. Grounded feet retain their placement;
// only the first half of a swing can adjust its touchdown target.
class FootstepFeedback {
public:
    FootstepFeedback(double cycle, double omega, Vector forward, Vector lateral,
                     double target_width)
        : cycle_(cycle), omega_(omega), forward_(forward), lateral_(lateral),
          target_width_(target_width) {}

    Vector offset(int side, double seconds) const {
        for (auto it = steps_[side].rbegin(); it != steps_[side].rend(); ++it) {
            if (seconds < it->begin) continue;
            const double u = std::clamp((seconds-it->begin)/(it->end-it->begin), 0.0, 1.0);
            const double blend = u*u*u*(10+u*(-15+6*u));
            Vector result{};
            for (int i=0; i<3; ++i) result[i] = it->from[i]+blend*(it->to[i]-it->from[i]);
            return result;
        }
        return {};
    }

    void update(double seconds, double dt, double commanded_velocity,
                const Vector& com, const Vector& velocity,
                const Vector& reference, const Vector& reference_velocity, double measured_width,
                double planned_width_delta = 0) {
        if (seconds < 0 || dt <= 0) return;
        const int lap = static_cast<int>(std::floor(seconds/cycle_));
        const double phase = seconds/cycle_-lap;
        Vector error{};
        for (int i=0; i<2; ++i)
            error[i] = com[i]+velocity[i]/omega_-reference[i]-reference_velocity[i]/omega_;
        for (int side=0; side<2; ++side) {
            const double begin = side == 0 ? .15 : .65;
            const double end = side == 0 ? .45 : .95;
            if (phase < begin || phase >= end) continue;
            const double start = (lap+begin)*cycle_;
            if (steps_[side].empty() || steps_[side].back().begin != start) {
                const auto from = offset(side, start);
                steps_[side].push_back({start, (lap+end)*cycle_, from, from,
                    (side==0 ? 1 : -1)*(target_width_+planned_width_delta-measured_width)});
            }
            // Freeze early enough for the quintic landing to reach zero velocity.
            if ((phase-begin)/(end-begin) >= .5) continue;
            auto& swing = steps_[side].back();
            const auto support = offset(1-side, seconds);
            const double speed_error = commanded_velocity-dot(velocity, forward_);
            const double forward = std::clamp(.3*dot(error,forward_)+.12*.5*cycle_*speed_error, -.025, .025);
            // Correct measured separation at liftoff, including joint tracking error.
            const double lateral = dot(swing.from,lateral_)+swing.width_correction+
                std::clamp(.3*dot(error,lateral_), -.02, .02);
            Vector desired{};
            for (int i=0; i<2; ++i) desired[i] =
                (dot(support,forward_)+forward)*forward_[i]+lateral*lateral_[i];
            // The relative adjustment stays inside a short, non-crossing step.
            const auto relative = subtract(desired,swing.from);
            const double dx = std::clamp(dot(relative,forward_), -.04, .04);
            const double dy = std::clamp(dot(relative,lateral_), -.025, .025);
            for (int i=0; i<2; ++i) {
                desired[i] = swing.from[i]+dx*forward_[i]+dy*lateral_[i];
                swing.to[i] += std::clamp(desired[i]-swing.to[i], -.06*dt, .06*dt);
            }
            max_adjustment_ = std::max(max_adjustment_, std::sqrt(dot(subtract(swing.to,swing.from),subtract(swing.to,swing.from))));
        }
    }

    void reset() { for (auto& steps : steps_) steps.clear(); }
    double maxAdjustment() const { return max_adjustment_; }

private:
    struct Step { double begin, end; Vector from, to; double width_correction; };
    double cycle_, omega_;
    Vector forward_, lateral_;
    double target_width_;
    std::array<std::vector<Step>,2> steps_;
    double max_adjustment_ = 0;
};
} // namespace frs::base
