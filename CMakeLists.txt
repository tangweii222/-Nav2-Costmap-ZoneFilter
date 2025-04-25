cmake_minimum_required(VERSION 3.5)
project(zone_filter)

find_package(ament_cmake REQUIRED)

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
