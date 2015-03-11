cmake_minimum_required(VERSION 2.8.3)
project(walking_group_recovery)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    actionlib
    actionlib_msgs
    strands_navigation_msgs
    monitored_navigation
    strands_monitored_nav_states
    smach
    smach_ros
    mary_tts
    rospy
    pygame_managed_player
    mongodb_media_server
)

catkin_python_setup()

add_service_files(
    FILES
    ToggleWalkingGroupRecovery.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

#############
## Install ##
#############
install(PROGRAMS
    scripts/toggle_walking_group_recovery_server.py
#    src/walking_group_recovery/walking_group_recovery_state.py
#    src/walking_group_recovery/walking_group_nav_states.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
