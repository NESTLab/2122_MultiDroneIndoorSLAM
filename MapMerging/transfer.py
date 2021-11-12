from __future__ import annotations # Allows classes to type hint their own class.
import numpy as np
import hough_sift_mapmerge as merge
import time, socket, threading

CHUNK_SIZE = 1024               # 1024 bytes
BANDWIDTH_IN_BYTES = 25000000   # 25Mb
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

# ************* UNUSED
class Network:
    def __init__(self, latency, bandwidth) -> None:
        self._latency = latency
        self._bandwidth = bandwidth
        self._bytesSent = 0
        self._timeCreated = time.time()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send(self, bytes) -> int:
        now = time.time()
        connectionDuration = now - self._timeCreated
        self._bytesSent += len(bytes)
        # How long should it have taken to send how many bytes we've sent with our
        # given bandwidth limitation?
        requiredDuration = self._bytesSent / self._bandwidth
        time.sleep(max(requiredDuration - connectionDuration, self._latency))
        return self._socket.send(bytes)
    
    def connect(self, address) -> None:
        self._socket.connect(address)
    
    def recv(self, bytes) -> None:
        self._socket.recv(bytes)

class Robot:
    """Simulated UAV robot"""
    def __init__(self, name, map) -> None:
        self.name = name
        self.map = map
    
    def send_map(self, sock) -> None:
        con, _ = sock.accept()
        con.sendall(self.map.to_bytes())
        con.close()

    def recv_map(self, sock) -> Map:
        sock.connect((IP_ADDRESS, PORT))
        payload = b''
        while True:
            data = sock.recv(CHUNK_SIZE)
            if data:
                payload += data
            else:
                break
        m = self.map.from_bytes(payload)

class Strategy:
    """Strategy for transfering occupancy grid between robots"""
    def __init__(self, name, r1, r2) -> None:
        self.name = name
        self.robots = [r1, r2]
    
    # Simulate data transfer from robot1 -> robot2
    def transfer(self, r1, r2) -> None:
        # Server
        sock1 = socket.socket()
        sock1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock1.bind(('', PORT))
        sock1.listen()
        # Client
        sock2 = socket.socket()
        # Create send & recieve maps in independent threads
        t1 = threading.Thread(target=r1.send_map, args=(sock1,))
        t2 = threading.Thread(target=r2.recv_map, args=(sock2,))
        # Start both threads
        t1.start()
        t2.start()
        # Wait for both threads to finish
        t1.join()
        t2.join()
        # Close sockets
        sock1.close()
        sock2.close()

    # Syncronize the maps of both robots
    def sync(self) -> None:
        r1, r2 = self.robots[0], self.robots[1]
        self.transfer(r1, r2)
        self.transfer(r2, r1)

class Evalutation():
    def __init__(self, strat) -> None:
        self.strat = strat
        self.duration = 0 # ms
        self.megabytes_transfered = 0
        self.accuracy = 0
    
    def start(self) -> Evalutation:
        start = time.time()
        strat.sync()
        end = time.time()
        self.duration = (end - start) / 1000
        return self

    def __repr__(self) -> str:
        return """###############################################
        Evaluation of "{}"

Duration: \t\t{} ms
Data Transfered: \t{} Mb
Accuracy: \t\t{}
""".format(self.strat.name, self.duration, self.megabytes_transfered, self.accuracy)

if __name__ == "__main__":
    m1, m2 = merge.TRAIN_FILENAMES[0], merge.TRAIN_FILENAMES[1]
    net = Network(LATENCY_IN_SECONDS, BANDWIDTH_IN_BYTES)
    rob_a = Robot('A', Map(merge.load_mercer_map(m1)))
    rob_b = Robot('B', Map(merge.load_mercer_map(m2)))
    strat = Strategy("testing", rob_a, rob_b)

    e = Evalutation(strat).start()
    print(e)
    

""" TODO
- Add metrics in evalutation function
- Create the full map transfer non-stategy
- Test mock network -> (maybe try the unused Network implimentation)
- Create the Equality and Error Correction startegy
- Create the DAG strategy
"""