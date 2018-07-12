cmake_minimum_required(VERSION 2.8.3)
project(robot_auto_docking)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_auto_docking
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#set(EXECUTABLE_OUTPUT_PATH  ../../devel/lib/${PROJECT_NAME})
#set(EXECUTABLE_OUTPUT_PATH /home/leishen/catkin_zz/devel/lib/robot_auto_docking)

add_executable(robot_dock src/robot_dock.cpp)
target_link_libraries(robot_dock ${catkin_LIBRARIES} )

add_executable(robot_dock_v2 src/robot_dock_v2.cpp)
target_link_libraries(robot_dock_v2 ${catkin_LIBRARIES} )

add_executable(robot_dock_v1 src/robot_dock_v1.cpp)
target_link_libraries(robot_dock_v1 ${catkin_LIBRARIES} )


install(TARGETS robot_dock_v1 robot_dock_v2 robot_dock
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
