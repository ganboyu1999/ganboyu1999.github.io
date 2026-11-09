cmake_minimum_required(VERSION 3.20)
project(python_thumbs_up LANGUAGES CXX)
# Python 业务无需编译；C++ 插件需要先构建这层 JSON/C 适配库。
add_subdirectory(pose_planner)
