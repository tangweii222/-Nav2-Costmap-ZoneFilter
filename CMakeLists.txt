cmake_minimum_required(VERSION 3.5)
project(zone_filter)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/zone_filter.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  std_msgs
  tf2_ros
  nav2_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d zone_filter.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

include_directories(include)
install(DIRECTORY include/ DESTINATION include)
install(FILES zone_filter.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch params maps DESTINATION share/${PROJECT_NAME})

ament_package()
