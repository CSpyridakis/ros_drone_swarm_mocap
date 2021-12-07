
TARGET=OUT
SRC=main.cpp \
	../../src/station/trilateration.cpp

PROJECT_NAME=ros_drone_swarm_mocap
CATKIN_WS=catkin_ws
CATKIN_POS=${HOME}

INCLUDE=-I../../include/${PROJECT_NAME} \
		-I/opt/ros/${ROS_DISTRO}/include \
		-I${CATKIN_POS}/${CATKIN_WS}/devel/include/

all:
	g++ -o ${TARGET} ${INCLUDE} ${SRC} 

clean:
	rm ${TARGET}