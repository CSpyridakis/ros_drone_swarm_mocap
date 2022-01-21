cmake_minimum_required(VERSION 2.8)

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  # -Werror
)

set(CATKIN_WS catkin_ws)
set(CATKIN_POS ${HOME})

project( ros_drone_swarm_mocap )
include_directories( ../../include/${PROJECT_NAME} )
include_directories( ../../lib/ )
include_directories( ../../../../devel/include/ )
include_directories( /opt/ros/noetic/include )
include_directories( /usr/include/ )

add_definitions(-DTESTING_MULTILATERATION)

set(CMAKE_BUILD_TYPE Debug)

# =================== Change these options ===================
set(EXEC_NAME OUT)
set(FilesToInclude 
main.cpp
../../src/station/trilateration.cpp
)
add_executable(${EXEC_NAME} ${FilesToInclude})
