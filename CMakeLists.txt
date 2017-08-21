cmake_minimum_required(VERSION 2.8.3)
project(ar_helipad)

find_package(catkin REQUIRED COMPONENTS
    tf2_msgs
    visualization_msgs
    ar_track_alvar
    ar_track_alvar_msgs)

catkin_package()

include_directories()
