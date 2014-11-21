cmake_minimum_required(VERSION 2.8.3)
project(backoff_behaviour)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    actionlib
    actionlib_msgs
    geometry_msgs 
    message_generation
    move_base_msgs 
    roscpp 
    rospy 
    sensor_msgs
    std_msgs 
    tf
    )

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
    DIRECTORY action 
	FILES Backoff.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(backoff_behaviour src/backoff.cpp)

add_dependencies(backoff_behaviour ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries (backoff_behaviour  ${catkin_LIBRARIES})

#############
## Install ##
#############

 install(TARGETS backoff_behaviour
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
