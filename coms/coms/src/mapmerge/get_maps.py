from coms.sim import Sim, is_sim_network_running, launch_sim_network, terminate_sim_network
import matplotlib.pyplot as plt
from typing import Dict, List
import pickle
import numpy as np
import time

DEFAULT_NET_SIM_CONFIG = "/root/catkin_ws/src/coms/ros-net-sim/example/cfg/testing.yaml"
DEFAULT_NET_SIM_LAUNCH_FILE = "/root/catkin_ws/src/coms/ros-net-sim/example/launch/testing.launch"
LAUNCH_CONFIG_LOCAL_IPS = ["192.168.0.1", "192.168.0.2", "192.168.0.3", "192.168.0.4"]
ROBOT_NAMES = ["robot_1", "robot_2", "robot_3", "robot_4"]
LOOPBACK_ADDRESS = '127.0.0.1'
LAUNCH_NET_OBJ = None

def show_array_of_maps(maps) -> None:
    fig, axes = plt.subplots(nrows=1, ncols=len(maps))
    for i in range(len(maps)):
        axes[i].imshow(maps[i], cmap="gray")
    plt.show()

def start_network() -> any:
    return launch_sim_network(DEFAULT_NET_SIM_LAUNCH_FILE, LAUNCH_CONFIG_LOCAL_IPS)

def stop_network(launch) -> None:
    terminate_sim_network(launch, LAUNCH_CONFIG_LOCAL_IPS)

def get_raw_maps() -> dict:
    return pickle.load(open("/root/catkin_ws/src/coms/coms/src/mapmerge/unexplored_intel.pkl", "rb"))

def gen_robots(maps: List[np.ndarray]) -> dict:
    result: Dict[str,Sim] = {}
    for idx, ip in  enumerate(LAUNCH_CONFIG_LOCAL_IPS):
        ns = "/" + ROBOT_NAMES[idx] + "/"
        s = Sim(address = ip, net_sim_launch_file=DEFAULT_NET_SIM_LAUNCH_FILE, net_config="testing.yaml", namespace=ns, local_map=maps[idx])
        result[ROBOT_NAMES[idx]] = s
    return result

def start_robots(robots: Dict[str,Sim]) -> None:
    for robot_name in robots:
        s = robots[robot_name]
        s.start()

def stop_robots(robots: Dict[str,Sim]) -> None:
    for robot_name in robots:
        s = robots[robot_name]
        s.stop()

def compute_score(map: np.ndarray) -> int:
    score = 0
    for row in map:
        for cell in row:
            # NOT unknown
            if cell != 127:
                score += 1
    return score

# Options: 0.95 | 0.9 | 0.85 | 0.6 | 0.5 | 0.4 | 0.1
def experiment(overlap: float) -> None:
    map_dict = get_raw_maps()
    robot_maps = map_dict[overlap]
    robot_maps = robot_maps[0:4]
    robots = gen_robots(robot_maps)

    # launch = start_network()
    start_robots(robots)
    time.sleep(10)
    stop_robots(robots)
    # stop_network(launch)

    print("\n\n\nchange!")
    n_maps = []
    for idx, map in enumerate(robot_maps):
        r_name = ROBOT_NAMES[idx]
        robo = robots[r_name]
        old = 0
        n = robo.local_score
        n_maps.append(robo.local_map)
        print("Robot {0} had {1} -> {2}".format(r_name, old, n))

    # show_array_of_maps(n_maps)

def main() -> None:
    experiment(0.1)





main()
