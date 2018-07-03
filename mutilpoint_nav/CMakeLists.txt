cmake_minimum_required(VERSION 2.8.3)
project(mutilpoint_nav)

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  ecl_exceptions
  ecl_threads
  ecl_time
)

# pkg-config support
find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH /home/leishen/catkin_zz/devel/lib/mutilpoint_nav)

add_executable(MutilPoint_nav src/Mutilpoint_nav.cpp src/yaml_parser.cpp)
target_link_libraries(MutilPoint_nav  ${catkin_LIBRARIES} yaml-cpp)
