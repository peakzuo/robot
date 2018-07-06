cmake_minimum_required(VERSION 2.8.3)
project(agv_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agv_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(agv_control src/agv_control.cpp)
target_link_libraries(agv_control  ${catkin_LIBRARIES} )

add_executable(agv_arm src/AGV_raise_and_down.cpp)
target_link_libraries(agv_arm  ${catkin_LIBRARIES} )


## Mark executables and/or libraries for installation
install(TARGETS agv_arm
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
