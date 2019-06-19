cmake_minimum_required( VERSION 2.8.3 )

project( syrena_ar_track )

find_package( catkin REQUIRED COMPONENTS 
    ar_track_alvar_msgs
    rospy
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy ar_track_alvar_msgs sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
