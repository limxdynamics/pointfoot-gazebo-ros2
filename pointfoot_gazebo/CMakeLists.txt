cmake_minimum_required(VERSION 3.5)
project(pointfoot_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(limxsdk_sim REQUIRED)

link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

## Declare a cpp library

add_library(poinfoot_gazebo_hardware_plugin SHARED src/PointFootHWSim.cpp)

target_include_directories(poinfoot_gazebo_hardware_plugin PRIVATE include)

ament_target_dependencies(poinfoot_gazebo_hardware_plugin
  angles
  gazebo_dev
  hardware_interface
  rclcpp
  gazebo_ros2_control
  realtime_tools
  limxsdk_sim
)

install(TARGETS
  poinfoot_gazebo_hardware_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp gazebo_ros2_control)
ament_export_libraries(poinfoot_gazebo_hardware_plugin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(gazebo_ros2_control poinfoot_gazebo_hardware_plugin.xml)

ament_package()
