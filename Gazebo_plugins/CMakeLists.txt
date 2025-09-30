cmake_minimum_required(VERSION 3.0.2)
project(rotor_wrench_plugin)

# Buscar Gazebo
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(RotorWrenchPlugin SHARED RotorWrenchPlugin.cc)
target_link_libraries(RotorWrenchPlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

