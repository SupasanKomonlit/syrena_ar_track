cmake_minimum_required( VERSION 2.8.3 )

project( syrena_ar_track )

find_pacakge( catkin REQUIRED COMPONENTS 
    ar_track_alvar_msgs
    rospy
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp ar_track_alvar_msgs sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
