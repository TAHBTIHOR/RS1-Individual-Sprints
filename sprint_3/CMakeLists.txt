cmake_minimum_required(VERSION 3.5)
project(sprint_3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable
add_executable(cylinder_detector src/cylinder_detector.cpp)

# Link against the ROS 2 libraries
ament_target_dependencies(cylinder_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs 
)

# Install the node
install(TARGETS
  cylinder_detector
  DESTINATION lib/${PROJECT_NAME})

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package
ament_package()
