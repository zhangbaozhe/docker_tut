# Docker Tutorial

I use reveal.js as the presentation slides. You can open `index.html` in the root directory in your browser to see the slides. 

The directory `docker_tut` contains several Dockerfiles that I need in the slide show.

## Setup

Most of the docker images in this repo need `nvidia-docker` support and assume the user uses a Linux PC with Nvidia GPU. 

If you want to run the samples in the slides, you need to install `nvidia-docker` package. 

### Install Docker

Follow the official document. 
https://docs.docker.com/engine/install/

### Install `nvidia-docker`

Links: 
  - https://github.com/NVIDIA/nvidia-container-toolkit
  - https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

```text
sudo apt-get update \
    && sudo apt-get install -y nvidia-container-toolkit-base

sudo apt-get install -y nvidia-container-toolkit 

sudo nvidia-ctk runtime configure --runtime=docker

sudo systemctl restart docker
```

### Pull required images

Just run 
```text
bash pull.bash
```
to pull the following images
```text
docker pull osrf/ros:noetic-desktop-full
docker pull nvidia/cuda:11.6.2-base-ubuntu20.04
docker pull zhangbaozhe/ros:noetic
docker pull zhangbaozhe/ros_xrdp:noetic
docker pull zhangbaozhe/ros_nvidia:noetic
docker pull zhangbaozhe/ros_nvidia_px4:noetic
docker pull zhangbaozhe/ros_nvidia_px4:noetic_simulation
docker pull zhangbaozhe/ros_nvidia_px4:noetic_controller
```