# Docker Tutorial Support Files

This directory contains the Dockerfiles, docker-compose files, and necessary scripts that I need in the slides. 

## Docker Images

All the images needed in the slides are uploaded to the Docker Hub.
Below shows the corresponding Dockerfile for each image: 
- `zhangbaozhe/ros:noetic` => `./docker_build_image_case-0-1/Dockerfile`
- `zhangbaozhe/ros_xrdp:noetic` => `./docker_build_image_case-0-3/Dockerfile`
- `zhangbaozhe/ros_nvidia:noeitc` => `./docker_build_image_case-0-11/Dockerfile`
- `zhangbaozhe/ros_nvidia_px4:noetic` => `./docker_px4_gazebo_case-1/docker_px4_gazebo/Dockerfile_base`
- `zhangbaozhe/ros_nvidia_px4:noetic_simulation` => `./docker_px4_gazebo_case-1/docker_px4_gazebo/Dockerfile`
- `zhangbaozhe/ros_nvidia_px4:noetic_controller` => `./docker_px4_gazebo_case-1/docker_controller/Dockerfile`

## Sample Docker Compose Files

The `docker-compose.yml` files shown in case 2 are in the two directories: 
- `./docker_distributed_sim_case-2/one/docker-compose.yml`
- `./docker_distributed_sim_case-2/ws/docker-compose.yml` (along with Python script)