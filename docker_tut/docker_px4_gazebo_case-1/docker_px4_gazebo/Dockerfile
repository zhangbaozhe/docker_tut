# zhangbaozhe/ros_nvidia_px4:noetic
FROM px4io/px4-dev-ros-noetic:latest
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

# Install packages
RUN apt-get update \
 && apt-get install -y locales lsb-release gnupg2
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN apt install -y git
RUN cd / && git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
 && bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && cd PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo -j1 
RUN apt-get update && apt-get install --assume-yes ros-noetic-mavros ros-noetic-joy ros-noetic-octomap-ros ros-noetic-octomap-msgs \
    && cd /opt/ros/noetic/lib/mavros && sudo ./install_geographiclib_datasets.sh \
    && apt-get install -y -qq ros-noetic-vrpn ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control 

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
