cmake_minimum_required(VERSION 2.8.3)
project(robot_auto_docking)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH  ../../devel/lib/${PROJECT_NAME})
#set(EXECUTABLE_OUTPUT_PATH /home/leishen/catkin_zz/devel/lib/robot_auto_docking)

add_executable(robot_dock src/robot_dock.cpp)
target_link_libraries(robot_dock ${catkin_LIBRARIES} )
