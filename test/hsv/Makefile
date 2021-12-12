# -------------------------------------------------
# Only modify these options
# -------------------------------------------------

TARGET=OUT
SRC=main.cpp 
	# ../../src/worker/misc.cpp \
	# ../../src/worker/hsvDetection.cpp

# -------------------------------------------------
# DO NOT MODIFY THE OPTIONS BELOW 
# -------------------------------------------------

# CFLAGS = -std=c++11
# OPENCV = pkg-config --cflags --libs opencv4
# LIBS = $(OPENCV)

# OPENCV = `pkg-config opencv4 --cflags --libs`
# LIBS = $(OPENCV)

# CFLAGS = $(pkg-config --cflags opencv4)
# LIBS = $(pkg-config --libs opencv4)

CXXFLAGS = -std=c++11 $(shell pkg-config --cflags opencv4)
LDLIBS=$(shell pkg-config --libs opencv4)

PROJECT_NAME=ros_drone_swarm_mocap
CATKIN_WS=catkin_ws
CATKIN_POS=${HOME}

INCLUDE=-I../../include/${PROJECT_NAME} \
		-I/opt/ros/${ROS_DISTRO}/include \
		-I${CATKIN_POS}/${CATKIN_WS}/devel/include/

all:
	g++ -o ${TARGET} ${INCLUDE} ${CFLAGS} ${LIBS} ${SRC} 

clean:
	rm -f ${TARGET}