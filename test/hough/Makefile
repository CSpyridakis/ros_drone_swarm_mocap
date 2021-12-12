# -------------------------------------------------
# Only modify these options
# -------------------------------------------------

TARGET=OUT
SRC=main.cpp \
	../../src/worker/misc.cpp \
	../../src/worker/houghDetection.cpp

# -------------------------------------------------
# DO NOT MODIFY THE OPTIONS BELOW 
# -------------------------------------------------

PROJECT_NAME=ros_drone_swarm_mocap
CATKIN_WS=catkin_ws
CATKIN_POS=${HOME}

INCLUDE=-I../../include/${PROJECT_NAME} \
		-I/opt/ros/${ROS_DISTRO}/include \
		-I${CATKIN_POS}/${CATKIN_WS}/devel/include/ \
		-I/usr/local/include/opencv4/

all:
	g++ -o ${TARGET} ${INCLUDE} ${SRC} 

clean:
	rm -f ${TARGET}