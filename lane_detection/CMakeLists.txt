cmake_minimum_required(VERSION 3.5)
project(lane_detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# target_include_directories(video_publisher_node PRIVATE
#     ${OpenCV_INCLUDE_DIRS}
#     ${rclcpp_INCLUDE_DIRS}
#     ${cv_bridge_INCLUDE_DIRS}
#     ${sensor_msgs_INCLUDE_DIRS}
# )
# target_link_libraries(video_publisher_node
#     ${OpenCV_LIBS}
#     ${rclcpp_LIBRARIES}
#     ${cv_bridge_LIBRARIES}
#     ${sensor_msgs_LIBRARIES}
# )

add_executable(lane_detection_node src/lane_detection_node.cpp)
# add_executable(video_publisher_node src/test.cpp)
ament_target_dependencies(lane_detection_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)
# ament_target_dependencies(video_publisher_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  lane_detection_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
