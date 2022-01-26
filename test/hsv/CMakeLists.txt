cmake_minimum_required(VERSION 2.8)

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  # -Werror
)

project( ros_drone_swarm_mocap )
set(CATKIN_WS catkin_ws)
set(CATKIN_POS ${HOME})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ../../include/${PROJECT_NAME} )
include_directories( ../../lib/ )
include_directories( ../../../../devel/include/ )
include_directories( /opt/ros/noetic/include )
include_directories( /opt/ros/melodic/include )

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DTESTING_HSV)

# =================== Change these options ===================
set(EXEC_NAME OUT)
set(FilesToInclude 
	main.cpp
	../../src/worker/misc.cpp
	../../src/worker/hsvDetection.cpp
	../../src/worker/extendDistAng.cpp
	../../src/worker/frequency.cpp
	../../lib/distance-angle/distance-angle.cpp
	../../lib/statistics/performance.cpp
)
add_executable(${EXEC_NAME} ${FilesToInclude})
target_link_libraries( ${EXEC_NAME} ${OpenCV_LIBS} )