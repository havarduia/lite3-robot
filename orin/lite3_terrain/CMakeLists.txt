cmake_minimum_required(VERSION 3.8)
project(lite3_terrain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(terrain_level src/terrain_level.cpp)
target_include_directories(terrain_level PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(terrain_level rclcpp sensor_msgs nav_msgs geometry_msgs)
install(TARGETS terrain_level DESTINATION lib/${PROJECT_NAME})

ament_package()
