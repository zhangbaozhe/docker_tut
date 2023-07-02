R=$1
W=$2

docker run -it --net=temp --ip 172.18.0.11 --runtime=nvidia --gpus all --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --rm -v ./csv/:/simulation_ws/src/control_demo/csv/ zhangbaozhe/ros_nvidia_px4:noetic_controller bash -i -c "cd simulation_ws && bash start.bash 172.18.0.11 1 $R $W"