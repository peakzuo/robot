cmake_minimum_required(VERSION 2.8.3)
project(clc)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_core
        )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
   
    CATKIN_DEPENDS
        roscpp
        nav_core
)

add_executable(clc src/clc.cpp)

#add_dependencies(clc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(clc
    ${catkin_LIBRARIES}
    )

install(TARGETS clc
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY launch
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)

#install(FILES bgp_plugin.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


