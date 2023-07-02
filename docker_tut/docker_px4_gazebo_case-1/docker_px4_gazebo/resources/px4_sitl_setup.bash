#!/bin/bash

if [ "$#" != 1 ]; then 
  echo -e "Usage: source px4_sitl_setup.bash <px4_dir>"
  return 1
fi

SRC_DIR=$1
BUILD_DIR=$SRC_DIR/build/px4_sitl_default

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo-classic:${BUILD_DIR}/build_gazebo-classic/devel/lib

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${SRC_DIR}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${SRC_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
