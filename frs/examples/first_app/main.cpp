#include <sdk.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

int main() {
    try {
        auto robot = frs::connect();
        const auto status = robot->system().status();
        const auto hardware = robot->capabilities().hardware("whole_body");
        const auto state = robot->state().snapshot();
        const auto count = hardware.parameters.at("joint_count").asNumber();
        if (state.quality != "good" || state.age.count() < 0 || state.age.count() > 100'000'000)
            throw std::runtime_error("state is unavailable or stale");
        if (!std::isfinite(count) || count <= 0 || count != std::floor(count) ||
            count != static_cast<double>(state.robot.joint_position.size()))
            throw std::runtime_error("joint position dimension does not match capability");
        for (const auto position : state.robot.joint_position)
            if (!std::isfinite(position)) throw std::runtime_error("joint position contains NaN or Inf");
        std::cout << frs::json::stringify(frs::json::Value::Object{
            {"health", status.health}, {"product_model", status.product_model},
            {"control_owner", status.control_owner}, {"joint_count", count},
            {"quality", state.quality}, {"age_ns", static_cast<double>(state.age.count())}
        }) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
