cmake_minimum_required(VERSION 3.15...3.28)

project(Robot_Arm LANGUAGES CXX)

add_executable(robot
    src/kinematics_base.h
    robot_arm_6dof_kinematics.h
    robot_arm_6dof_kinematics.cpp
    test.cpp
)


add_library(eigen INTERFACE)

target_include_directories(eigen INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/reference/eigen
)

target_link_libraries(robot
    PRIVATE eigen
)
