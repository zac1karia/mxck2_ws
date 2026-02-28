cmake_minimum_required(VERSION 3.5)
project(follow_the_gap_v0)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacle_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# add follow_the_gap node executable
add_executable(follow_the_gap
  "src/corner.cpp"
  "src/follow_the_gap.cpp"
  "src/gap.cpp"
  "src/lidar_data.cpp"
  "src/main.cpp"
  "src/obstacle.cpp"
)
ament_target_dependencies(follow_the_gap
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  obstacle_msgs
  tf2_geometry_msgs
)

# install nodes
install(
  TARGETS follow_the_gap
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
