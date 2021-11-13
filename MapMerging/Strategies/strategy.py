from __future__ import annotations # Allows classes to type hint their own class.
import numpy as np
import time, socket, threading, timeit
from hough_sift_mapmerge import experiment

NUM_MERGES_TO_TEST = 5          # determins number of map merges to average

CHUNK_SIZE = 2312               # 2312 bytes | Reasoning: The MTU of wireless networks is 2312 bytes
BANDWIDTH_IN_BYTES = 25000000   # 25Mbps
LATENCY_IN_SECONDS = 0.02       # 20ms
PORT = 9090
IP_ADDRESS = "127.0.0.1"

class Map():
    def __init__(self, map) -> None:
        self.map = map
        self.num_rows = len(map)
        self.num_col = len(map[0])

    def __repr__(self) -> str:
        return str(self.map)
    
    def to_bytes(self) -> bytes:
        return self.map.tobytes()
    
    def from_bytes(self, data) -> Map:
        m = np.frombuffer(data, dtype=np.uint8)
        m = m.reshape(self.num_rows, self.num_col)
        return Map(m)

class Network:
    def __init__(self, latency, bandwidth, sock = None) -> None:
        self._latency = latency
        self._bandwidth = bandwidth
        self._bytesSent = 0
        self._timeCreated = time.time()
        if sock is None:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self._socket = sock

    def send(self, bytes) -> int:
        now = time.time()
        connectionDuration = now - self._timeCreated
        self._bytesSent += len(bytes)
        requiredDuration = self._bytesSent / self._bandwidth
        time.sleep(max(requiredDuration - connectionDuration, self._latency))
        return self._socket.send(bytes)
    
    def sendall(self, bytes) -> int:
        return self._socket.sendall(bytes)
    
    def serve(self) -> None:
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('', PORT))
        self._socket.listen()
    
    def connect(self, address) -> None:
        self._socket.connect(address)
    
    def recv(self, bytes) -> bytes:
        return self._socket.recv(bytes)
    
    def close(self) -> None:
        self._socket.close()
    
    def accept(self) -> tuple[socket.SocketType, str]:
        return self._socket.accept()

class Robot:
    """Simulated UAV robot"""
    def __init__(self, name, map) -> None:
        self.name = name
        self.map = map

    def setup_network(self):
        self.net = Network(LATENCY_IN_SECONDS, BANDWIDTH_IN_BYTES)

    def get_bytes_sent(self) -> bytes:
        return self.net._bytesSent
    
    def send_map(self) -> None:
        self.net.serve()
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

class Strategy:
    """Strategy for transfering occupancy grid between robots"""
    def __init__(self, name, r1, r2) -> None:
        self.name = name
        self.robots = [r1, r2]
        self.bytes_transfered = 0
    
    # Simulate data transfer from robot1 -> robot2
    def transfer(self, r1, r2) -> bytes:
        r1.setup_network()
        r2.setup_network()
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
        r1, r2 = self.robots[0], self.robots[1]
        self.bytes_transfered += self.transfer(r1, r2)
        self.bytes_transfered += self.transfer(r2, r1)

class Evaluation():
    def __init__(self, strat) -> None:
        self.strat = strat
        self.duration = 0
        self.megabytes_transfered = 0
        self.accuracy = 0
    
    def start(self) -> Evaluation:
        start = time.time()
        self.strat.sync()
        end = time.time()
        self.duration = (end - start)
        self.megabytes_transfered = self.strat.bytes_transfered / 1000

        results = experiment(NUM_MERGES_TO_TEST)
        self.accuracy = results["SIFT_RESULTS"][0][0] * 100
        return self

    def __repr__(self) -> str:
        return """###############################################
        Evaluation of "{}"

Duration: \t\t{} seconds
Data Transfered: \t{} Mb
Accuracy (SIFT): \t{} %
""".format(self.strat.name, self.duration, self.megabytes_transfered, self.accuracy)
