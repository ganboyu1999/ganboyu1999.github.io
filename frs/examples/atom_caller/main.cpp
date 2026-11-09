#include <sdk.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "usage: atom_caller ATOM_ID\n"; return 1; }
    try {
        // 1. 普通应用连接 FRS，不使用 Atom 子进程的 start/on_command 运行时。
        auto robot = frs::connect();
        // 2. 分配并保存原 ID，再用相同 ID 发送 JSON 输入。
        const auto id = robot->atom().allocateRequest(argv[1]);
        std::cout << "request_id=" << id << std::endl;
        const std::string input = "{}"; // 点赞目标在模块内固定，调用方不传位置。
        const auto request = frs::atom::makeAtomCallRequest(id, argv[1], "run", input);
        // 回执成功表示受理；业务终态由下面的 taskResult 查询确认。
        const auto receipt = robot->atom().call(request);
        if (receipt.result_code() != 0) throw std::runtime_error(receipt.message());
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (std::chrono::steady_clock::now() < deadline) {
            // 3. 查询原任务的最新报告和历史，不重新提交命令。
            const auto result = robot->atom().taskResult(id, true);
            const auto* data = result.find("data");
            if (data && result.at("availability").asString() == "available") {
                const auto state = data->at("state").asString();
                if (state == "idle" || state == "failed" || state == "stopped") {
                    // 4. 终态还要核对 result_code；available 本身不表示任务成功。
                    if (data->at("result_code").asNumber() != 0)
                        throw std::runtime_error(data->at("message").asString());
                    std::cout << frs::json::stringify(result) << '\n';
                    return 0;
                }
            }
            const auto availability = result.at("availability").asString();
            if (availability == "expired" || availability == "not_found")
                throw std::runtime_error("task history is " + availability);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        // 超时保留原 ID，让调用方继续查结果，避免未知受理状态下重复执行。
        throw std::runtime_error("query this request_id before deciding whether to resubmit: " + id);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
