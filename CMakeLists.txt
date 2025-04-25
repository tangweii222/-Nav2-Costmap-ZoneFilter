cmake_minimum_required(VERSION 3.5)
project(zone_filter)

# 使用 C++14 或以上
set(CMAKE_CXX_STANDARD 14)

# 找到必要的套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

# 包含標頭檔路徑
include_directories(
  include
)

# 編譯 plugin 為 shared library
add_library(${PROJECT_NAME} SHARED
  src/zone_filter.cpp
)

# 連結依賴
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

# 宣告 pluginlib 對應的 XML 檔
pluginlib_export_plugin_description_file(nav2_costmap_2d zone_filter.xml)

# 安裝編譯產出
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安裝標頭與 plugin xml
install(DIRECTORY include/
  DESTINATION include/
)

install(FILES zone_filter.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安裝 Launch、params、maps 等資料夾
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
