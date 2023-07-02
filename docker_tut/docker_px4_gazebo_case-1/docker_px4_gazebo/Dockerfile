# zhangbaozhe/ros_nvidia_px4:noetic_simulation
FROM zhangbaozhe/ros_nvidia_px4:noetic

RUN mkdir -p simulation_ws/src && apt install -y python-is-python3 && pip install zmq
WORKDIR /simulation_ws/src
RUN git clone https://github.com/zhangbaozhe/new_swarm_bridge.git && \
    cd new_swarm_bridge && git checkout concurrent &&\
    cd ../.. && \
    . /opt/ros/noetic/setup.sh && catkin_make
WORKDIR /

COPY resources/* /simulation_ws/
