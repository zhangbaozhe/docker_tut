import sys
import numpy as np

HEAD = \
    """
version: '3'

services:
"""

NETWORKS_HEAD = \
    """
networks: 
   """


def generate_one_simulation(i, r, w):
    sim_subnet = i+20
    controller_subnet = sim_subnet

    simulation_str = \
        """
  sim{}: 
    image: zhangbaozhe/ros_nvidia_px4:noetic_simulation
    networks: 
      group{}: 
        ipv4_address: 172.{}.0.10
    expose: 
      - 5555
    volumes: 
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /etc/localtime:/etc/localtime
    environment:
      - DISPLAY
      - QT_X11_NO_MITSHM=1
    deploy: 
      resources: 
        limits: 
          cpus: '1'
        reservations: 
          devices: 
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    command: bash  -c "cd simulation_ws && bash start.bash 172.{}.0.10 0 False"
  mpc{}: 
    image: zhangbaozhe/ros_nvidia_px4:noetic_controller
    networks: 
      group{}: 
        ipv4_address: 172.{}.0.11
    expose: 
      - 5555
    volumes: 
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /etc/localtime:/etc/localtime
      - ./csv/:/simulation_ws/src/control_demo/csv/
    environment:
      - DISPLAY
      - QT_X11_NO_MITSHM=1
    deploy: 
      resources: 
        limits: 
          cpus: '1'
        reservations: 
          devices: 
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    command: bash  -c "cd simulation_ws && bash start.bash 172.{}.0.11 1 {} {}"
""".format(int(i),
           int(i), 
           str(sim_subnet),
           str(sim_subnet),
           int(i),
           int(i),
           str(controller_subnet),
           str(controller_subnet),
           r, w)

    return simulation_str

def generate_one_network(i):
    network_str = \
        """
  group{}: 
    driver: bridge
    ipam: 
      config: 
        - subnet: 172.{}.0.0/16
          gateway: 172.{}.0.1
        """.format(i, str(i+20), str(i+20))
    return network_str


if __name__ == '__main__':
    divide_num = int(sys.argv[1])
    r_max = float(sys.argv[2])
    w_max = float(sys.argv[3])
    r_step = r_max / divide_num
    w_step = w_max / divide_num
    r_array = np.linspace(0.0, r_max, divide_num)
    w_array = np.linspace(0.01, w_max+0.01, divide_num)
    result = HEAD
    for i, (r, w) in enumerate([(r, w) for r in r_array for w in w_array]):
        result += generate_one_simulation(i, r, w)
    result += NETWORKS_HEAD
    for i, (r, w) in enumerate([(r, w) for r in r_array for w in w_array]):
        result += generate_one_network(i)
    print(result)
    with open("docker-compose.yml", "w") as f:
        f.write(result)
