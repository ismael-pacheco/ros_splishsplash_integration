cmake_minimum_required(VERSION 3.0.2)
project(multi_drone_gazebo_plugin)

## Compilación C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Catkin y paquetes requeridos
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

## Gazebo
find_package(gazebo REQUIRED)

## Incluye headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

## Plugin de Gazebo
add_library(MultiDronePlugin SHARED src/MultiDronePlugin.cc)
target_link_libraries(MultiDronePlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(MultiDronePlugin ${catkin_EXPORTED_TARGETS})

## Nodo ROS externo que publica las posiciones deseadas
add_executable(pose_publisher_node src/pose_publisher_node.cpp)
target_link_libraries(pose_publisher_node ${catkin_LIBRARIES})
add_dependencies(pose_publisher_node ${catkin_EXPORTED_TARGETS})

## Instalación (opcional)
install(TARGETS MultiDronePlugin pose_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

