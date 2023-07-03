#!/bin/bash

# put this script in the workspace dir
# this is 0


IP=$1
ID=$2
R=$3
W=$4

source devel/setup.bash

trap 'kill -INT -$pid' INT
timeout 60 roslaunch start.launch ip:=$IP id:=$ID r:=$R w:=$W&
pid=$!
wait $pid