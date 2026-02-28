cmake_minimum_required(VERSION 3.5)
project(cartographer_slam)

find_package(ament_cmake REQUIRED)

# install config and launch files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
