cmake_minimum_required(VERSION 3.8)
project(dagana_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Gazebo
find_package(ament_cmake REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
include_directories(${gz-sim8_INCLUDE_DIRS})

# link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Build our plugin
add_library(dagana_gazebo_DaganaPlugin SHARED
    src/DaganaPlugin.cpp
)
target_link_libraries(dagana_gazebo_DaganaPlugin 
  ${gz-sim8_LIBRARIES}
  ${gz-plugin2_LIBRARIES}
)

install(DIRECTORY include launch src
    DESTINATION share/${PROJECT_NAME})

ament_package()