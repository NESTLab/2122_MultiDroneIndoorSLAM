from __future__ import annotations # Allows classes to type hint their own class.
import time, threading
from stale_coms.map import Map
from merge_utils import load_mercer_map
from experiments import paper_benchmark

NUM_MERGES_TO_TEST = 3          # determins number of map merges to average

class Scinario:
    """Scinario for transfering occupancy grid between robots"""
    def __init__(self, name, m1, m2, robot_type) -> None:
        self.bytes_transfered = 0
        self.name = name
        self.r1 = robot_type('A', Map(load_mercer_map(m1)))
        self.r2 = robot_type('B', Map(load_mercer_map(m2)))
    
    # Simulate data transfer from robot1 -> robot2
    def transfer(self, r1, r2) -> bytes:
        r1.setup_network()
        r2.setup_network()
        # Start server to ensure the client can connect
        r1.net.serve() 
        # Create send & recieve maps in independent threads
        t1 = threading.Thread(target=r1.send_data, args=())
        t2 = threading.Thread(target=r2.recv_data, args=())
        # Start both threads
        t1.start()
        t2.start()
        # Wait for both threads to finish
        t1.join()
        t2.join()
        # Close network sockets
        r1.close()
        r2.close()
        return r1.get_bytes_sent()
    
    # Syncronize the maps of both robots
    def sync(self) -> None:
        self.bytes_transfered += self.transfer(self.r1, self.r2)
        self.bytes_transfered += self.transfer(self.r2, self.r1)

class Evaluation():
    def __init__(self, scene) -> None:
        self.scene = scene
        self.duration = 0
        self.megabytes_transfered = 0
        self.accuracy = 0
    
    def start(self) -> Evaluation:
        start = time.time()
        self.scene.sync()
        end = time.time()
        self.duration = (end - start)
        self.megabytes_transfered = self.scene.bytes_transfered / 1000

        # Pairs together local and remote maps from both robots
        robot_map_pairs = [
            [self.scene.r1.map.map, self.scene.r2.remote_map.map],
            [self.scene.r1.remote_map.map, self.scene.r2.map.map]]
        # average SIFT results for both pairs
        for pair in robot_map_pairs: 
            results = paper_benchmark(n_iters=NUM_MERGES_TO_TEST, silent=True, maps=pair)
            self.accuracy += results["SIFT_RESULTS"][0][0] * 100
        self.accuracy = self.accuracy/len(robot_map_pairs)

        return self

    def __repr__(self) -> str:
        return """###############################################
        Evaluation of "{}"

Duration: \t\t{} seconds
Data Transfered: \t{} Mb
Accuracy (SIFT): \t{} %
""".format(self.scene.name, self.duration, self.megabytes_transfered, self.accuracy)
