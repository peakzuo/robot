cmake_minimum_required(VERSION 2.8.3)
project(ls_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES leishen_node
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)





#AUX_SOURCE_DIRECTORY(src DIR_SRCS)
#SET(TEST_MATH ${DIR_SRCS} )
#add_executable(ls_node ${TEST_MATH})
add_executable(ls_node src/ls_ros.cpp src/ls_diff.cpp src/ls_odometry.cpp src/package_analysis.cpp src/serial.cpp )
target_link_libraries(ls_node ${catkin_LIBRARIES})


add_executable(send_velocity src/send_velocity.cpp)
target_link_libraries(send_velocity ${catkin_LIBRARIES})
