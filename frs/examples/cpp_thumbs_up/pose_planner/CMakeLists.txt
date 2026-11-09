cmake_minimum_required(VERSION 3.20)
project(pose_planner LANGUAGES CXX)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
# 这里只构建跨语言适配层；实际规划算法来自配套 FBA 的插件。
add_library(pose_planner SHARED planner.cpp)
set_target_properties(pose_planner PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_compile_features(pose_planner PRIVATE cxx_std_17)
target_compile_options(pose_planner PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(pose_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/fba_contract)
target_link_libraries(pose_planner PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json ${CMAKE_DL_LIBS})
