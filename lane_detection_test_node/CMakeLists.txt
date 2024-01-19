# lane_detection_test_node/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(lane_detection_test_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Build
add_executable(lane_detection_test_node src/lane_detection_test_node.cpp)
ament_target_dependencies(lane_detection_test_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

install(TARGETS
  lane_detection_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
