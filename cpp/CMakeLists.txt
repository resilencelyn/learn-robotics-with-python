cmake_minimum_required(VERSION 2.8.3)

project(ROBOTICS_TOOLBOX)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_library(
  ${PROJECT_NAME}
  src/robotics_toolbox.cpp
)

add_executable(
  example src/example.cpp
)

target_link_libraries(
  example ${PROJECT_NAME}
)


