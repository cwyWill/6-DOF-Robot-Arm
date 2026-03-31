cmake_minimum_required(VERSION 3.20)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# add_library(eigen INTERFACE)
# target_include_directories(eigen INTERFACE
#   ${CMAKE_CURRENT_SOURCE_DIR}/reference/eigen
# )


add_executable(kinematics_node
  src/kinematics_node.cpp
  src/robot_arm_6dof_kinematics.cpp
)

target_include_directories(kinematics_node PUBLIC include)

target_link_libraries(kinematics_node PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${interfaces_TARGETS}
  Eigen3::Eigen
)

install(TARGETS
  kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
