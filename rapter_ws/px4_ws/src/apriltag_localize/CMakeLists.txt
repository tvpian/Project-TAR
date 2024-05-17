cmake_minimum_required(VERSION 3.8)
project(apriltag_localize)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED) 

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

add_executable(apriltag_localize_node src/apriltag_xcoord.cpp)
ament_target_dependencies(apriltag_localize_node
  rclcpp
  geometry_msgs
  sensor_msgs
  px4_msgs
  Eigen3
)

add_executable(apriltag_localize_simulation_node src/apriltag_xcoord_simulation.cpp)
ament_target_dependencies(apriltag_localize_simulation_node
  rclcpp
  geometry_msgs
  sensor_msgs
  px4_msgs
  Eigen3
  tf2_msgs
) 


add_executable(test_apriltag src/test_apriltag.cpp)
ament_target_dependencies(test_apriltag
  rclcpp
  geometry_msgs
  sensor_msgs
  px4_msgs
  Eigen3
  tf2_msgs
) 


install(TARGETS
  apriltag_localize_node
  apriltag_localize_simulation_node
  test_apriltag
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
