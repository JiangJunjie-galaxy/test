cmake_minimum_required(VERSION 2.8.3)
project(K_means_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -W -Wall -pedantic -fopenmp -fPIE -msse4.1 -g")
set(CMAKE_BUILD_TYPE "RELEASE")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL  REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)


add_executable(mcl
        src/main.cpp src/particle_filter.cpp
        )



target_link_libraries(mcl
        ${catkin_LIBRARIES}
        )



