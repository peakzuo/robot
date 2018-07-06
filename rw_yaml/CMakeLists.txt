cmake_minimum_required(VERSION 2.8.3)
project(rw_yaml)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/yaml-cpp
)

add_executable(read src/read_yaml.cpp)
target_link_libraries(read ${catkin_LIBRARIES} )
target_link_libraries(read yaml-cpp )
