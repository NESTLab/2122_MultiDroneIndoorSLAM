import numpy as np
import hough_sift_mapmerge as merge
import time, socket, threading

BANDWIDTH_IN_BYTES = 25000000   # 25Mb
LATENCY_IN_SECONDS = 0.02       # 20ms
PORT = 9090
IP_ADDRESS = "127.0.0.1"

class Net():
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

class Network:
    def __init__(self, latency, bandwidth):
        self._latency = latency
        self._bandwidth = bandwidth
        self._bytesSent = 0
        self._timeCreated = time.time()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send(self, bytes):
        now = time.time()
        connectionDuration = now - self._timeCreated
        self._bytesSent += len(bytes)
        # How long should it have taken to send how many bytes we've sent with our
        # given bandwidth limitation?
        requiredDuration = self._bytesSent / self._bandwidth
        time.sleep(max(requiredDuration - connectionDuration, self._latency))
        return self._socket.send(bytes)
    
    def connect(self, address):
        self._socket.connect(address)
        print("connected")
    
    def recv(self, bytes):
        self._socket.recv(bytes)

class Robot:
    """Simulated UAV robot"""
    def __init__(self, name, map, net):
        self.name = name
        self.map = map
        self.net = net
    
    def send_map(self, sock):
        con, _ = sock.accept()
        con.send(("from robot " + self.name).encode())
        con.close()

    def recv_map(self, sock):
        sock.connect((IP_ADDRESS, PORT))
        data = sock.recv(1024).decode()
        print(self.name, "recieved:", data)

class Strategy:
    """Strategy for transfering occupancy grid between robots"""
    def __init__(self, name, r1, r2):
        self.name = name
        self.robots = [r1, r2]
    
    # Simulate data transfer from robot1 -> robot2
    def transfer(self, r1, r2):
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
    def sync(self):
        r1, r2 = self.robots[0], self.robots[1]
        print("syncing robots", r1.name, "and", r2.name, "...")
        self.transfer(r1, r2)
        self.transfer(r2, r1)

# Display transfer time, merge accuracy, and bytes transmitted.
def evaluate(strat):
    print("evaluating")

if __name__ == "__main__":
    m1, m2 = merge.TRAIN_FILENAMES[0], merge.TRAIN_FILENAMES[1]
    net = Network(LATENCY_IN_SECONDS, BANDWIDTH_IN_BYTES)
    rob_a = Robot('A', merge.load_mercer_map(m1), net)
    rob_b = Robot('B', merge.load_mercer_map(m2), net)
    strat = Strategy("testing", rob_a, rob_b)
    strat.sync()