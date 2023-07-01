#!/bin/bash

worker_index=$1
worker_num=$2
step_size=$3
token=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

echo "Worker index is $worker_index"
echo "Worker number is $worker_num"
echo "Step size is $step_size"

cd ~/landing_ws/src

git checkout baozhe_simulator

git pull https://${token}@github.com/zhangbaozhe/mpc-relative-frame.git baozhe_simulator

cd ~/landing_ws

source /opt/ros/noetic/setup.bash

catkin_make -DCMAKE_BUILD_TYPE=Release -j1

source devel/setup.bash

cd ~/

mkdir data

cd data 

rosrun fast_relative_mpc num_fixed_point.py  $worker_index $worker_num $step_size



