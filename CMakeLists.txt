cmake_minimum_required(VERSION 3.16.3)
project(hybrid_astar)

# debug mdoe.
SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_STANDARD 14)
# This makes C++14 a requirement,
# and prevents a "decay" to C++98 when the compiler does not support C++14.
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
# This disables the use of compiler-specific extensions.
# For example, by default, CMake passes -std=gnu++14 to GCC on Linux.
# We want to build with -std=c++14. 
SET(CMAKE_CXX_EXTENSIONS OFF)

ADD_DEFINITIONS(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp  # Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp  # Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp   # Boris Lau, Christoph Sprunk, Wolfram Burgard
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_ros
    pcl_conversions
    std_msgs
    tf
)

find_package(PCL 1.13 REQUIRED)
SET(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.13/pcl)
add_definitions(${PCL_DEFINITIONS})

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include
    DEPENDS PCL
    CATKIN_DEPENDS 
    std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

add_executable(hybrid_astar src/main.cpp ${SOURCES})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES})

