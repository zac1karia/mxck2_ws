cmake_minimum_required(VERSION 3.5)
project(storage)

# find dependencies
find_package(ament_cmake REQUIRED)

# install files
install(
  DIRECTORY foxglove
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
