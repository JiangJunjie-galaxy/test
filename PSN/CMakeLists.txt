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
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pointsiam_ros
 CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
include_directories(mlpack REQUIRED)
include_directories(armadillo)


add_executable(laserscan_receive
        src/J_method.cpp
        )

#add_executable(laserscan_receive
#        src/velocity_motion.cpp
#        )

#add_executable(laserscan_receive
#        src/K_means_calculate_point.cpp
#       )
target_link_libraries(laserscan_receive
        ${catkin_LIBRARIES}
        #mlpack
        )

#
#add_executable(K_localization
#        src/J_method.cpp
#        )
#target_link_libraries(
#        K_localization
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#)

