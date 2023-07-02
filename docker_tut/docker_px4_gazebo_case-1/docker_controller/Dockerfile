# zhangbaozhe/ros_nvidia_px4:noetic_controller
FROM zhangbaozhe/ros_nvidia_px4:noetic

RUN mkdir -p simulation_ws/src && apt install -y python-is-python3 libgflags-dev libeigen3-dev && pip install zmq
WORKDIR /simulation_ws/src
RUN git clone https://github.com/zhangbaozhe/new_swarm_bridge.git && \
    cd new_swarm_bridge && git checkout concurrent
WORKDIR /simulation_ws/src
COPY resources/control_demo/ /simulation_ws/src/control_demo/
WORKDIR /simulation_ws
COPY resources/start.* /simulation_ws/
RUN . /opt/ros/noetic/setup.sh && catkin_make

WORKDIR /