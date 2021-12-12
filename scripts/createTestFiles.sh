#!/bin/bash

project_dir="${HOME}/catkin_ws/src/ros_drone_swarm_mocap"
experi_dir="${project_dir}/test/experiments/"
images_dir="images/"

files="angles.txt cpu.txt distance.txt hsvDetection.txt power.txt ram.txt time.txt"

cd ${experi_dir}
pwd
rm -rf ${files}
rm -rf ${images_dir}
touch ${files}
mkdir -p ${images_dir}