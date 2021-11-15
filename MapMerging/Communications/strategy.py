from __future__ import annotations # Allows classes to type hint their own class.
import time, socket, threading
from hough_sift_mapmerge import experiment, load_mercer_map
from Communications.map import Map
from Communications.network import Network, CHUNK_SIZE, IP_ADDRESS, PORT

NUM_MERGES_TO_TEST = 5          # determins number of map merges to average

class Full_Robot:
    """Transfering FULL map"""
    def __init__(self, name, map) -> None:
        self.name = name
        self.map = map

    def setup_network(self):
        self.net = Network()

    def get_bytes_sent(self) -> bytes:
        return self.net._bytesSent
    
    def send_map(self) -> None:
        con, _ = self.net.accept()
        connection = Network(self.net._latency, self.net._bandwidth, sock = con)
        map_data = self.map.to_bytes()
        map_chunks = [map_data[i:i+CHUNK_SIZE] for i in range(0, len(map_data), CHUNK_SIZE)]
        for chunk in map_chunks:
            connection.send(chunk)

        self.net._bytesSent = connection._bytesSent
        connection.close()

    def recv_map(self) -> Map:
        self.net.connect((IP_ADDRESS, PORT))
        payload = b''
        while True:
            data = self.net.recv(CHUNK_SIZE)
            if data:
                payload += data
            else:
                break
        m = self.map.from_bytes(payload)
    
    def close(self) -> None:
        self.net.close()

class DAG_Robot():
    """Transfering partial DAG of map"""
    def __init__(self, name, map) -> None:
        self.name = name
        self.map = map
    

    def setup_network(self):
        self.net = Network()

    def get_bytes_sent(self) -> bytes:
        return self.net._bytesSent
    
    def send_map(self) -> None:
        con, _ = self.net.accept()
        connection = Network(self.net._latency, self.net._bandwidth, sock = con)
        map_data = self.map.to_bytes()
        map_chunks = [map_data[i:i+CHUNK_SIZE] for i in range(0, len(map_data), CHUNK_SIZE)]
        for chunk in map_chunks:
            connection.send(chunk)

        self.net._bytesSent = connection._bytesSent
        connection.close()

    def recv_map(self) -> Map:
        self.net.connect((IP_ADDRESS, PORT))
        payload = b''
        while True:
            data = self.net.recv(CHUNK_SIZE)
            if data:
                payload += data
            else:
                break
        m = self.map.from_bytes(payload)

    def close(self) -> None:
        self.net.close()

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
        t1 = threading.Thread(target=r1.send_map, args=())
        t2 = threading.Thread(target=r2.recv_map, args=())
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

        results = experiment(NUM_MERGES_TO_TEST)
        self.accuracy = results["SIFT_RESULTS"][0][0] * 100
        return self

    def __repr__(self) -> str:
        return """###############################################
        Evaluation of "{}"

Duration: \t\t{} seconds
Data Transfered: \t{} Mb
Accuracy (SIFT): \t{} %
""".format(self.scene.name, self.duration, self.megabytes_transfered, self.accuracy)
