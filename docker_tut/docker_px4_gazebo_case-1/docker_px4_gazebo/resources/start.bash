#!/bin/bash

# put this script in the workspace dir
# this is 0


IP=$1
ID=$2

source devel/setup.bash
source px4_sitl_setup.bash /PX4-Autopilot

trap 'kill -INT -$pid' INT
timeout 60 roslaunch start.launch ip:=$IP id:=$ID > /dev/null &
pid=$!

sleep 10 && 
rosrun mavros mavsafety arm && 
rosrun mavros mavsys mode -c OFFBOARD
wait $pid


