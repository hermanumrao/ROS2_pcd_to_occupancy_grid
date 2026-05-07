cmake_minimum_required(VERSION 3.8)
project(map_3d_to_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# PCL (IMPORTANT)
find_package(PCL REQUIRED)

# -------------------------------
# Include directories
# -------------------------------
include_directories(include ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# -------------------------------
# Executable
# -------------------------------
add_executable(mapper src/mapper.cpp)

ament_target_dependencies(
  mapper
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros)

# Link PCL explicitly
target_link_libraries(mapper ${PCL_LIBRARIES})

# -------------------------------
# Install
# -------------------------------
install(TARGETS mapper DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/map_gui.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
