#!/bin/bash

# put this script in the workspace dir
# this is 0


IP=$1
ID=$2
GUI=$3

source devel/setup.bash
source px4_sitl_setup.bash /PX4-Autopilot

trap 'kill -INT -$pid' INT
timeout 60 roslaunch start.launch ip:=$IP id:=$ID gui:=$GUI&
pid=$!

sleep 10 
for i in {0..10..1}
do
  rosrun mavros mavsafety arm && 
  rosrun mavros mavsys mode -c OFFBOARD
done
wait $pid


