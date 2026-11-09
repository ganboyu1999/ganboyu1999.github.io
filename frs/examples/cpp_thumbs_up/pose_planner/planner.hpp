#pragma once
#include <cstddef>

// 教学工程自己的跨语言入口。输入、输出均为 UTF-8 JSON，输出内存由调用方管理。
// 返回 0 表示成功；1 表示错误 JSON；2 表示参数/缓冲区不足。异常不跨 C 边界。
extern "C" int thumbs_up_plan(const char* input, char* output, std::size_t capacity) noexcept;
