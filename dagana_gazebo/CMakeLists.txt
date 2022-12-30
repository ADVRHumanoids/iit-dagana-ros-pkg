cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(dagana_gazebo)

# Find Gazebo
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    gazebo_ros
)


catkin_package(
    INCLUDE_DIRS include
#   LIBRARIES rosee_gazebo_plugins
#   CATKIN_DEPENDS dynamic_reconfigure
    DEPENDS gazebo_ros
)

include_directories( include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Build our plugin
add_library(${PROJECT_NAME}_DaganaPlugin SHARED
    src/DaganaPlugin.cpp
)
target_link_libraries(${PROJECT_NAME}_DaganaPlugin 
    ${GAZEBO_LIBRARIES}
)
