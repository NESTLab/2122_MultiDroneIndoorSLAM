import time, socket

CHUNK_SIZE = 2312               # 2312 bytes | Reasoning: The MTU of wireless networks is 2312 bytes
BANDWIDTH_IN_BYTES = 25000000   # 25Mbps
LATENCY_IN_SECONDS = 0.02       # 20ms
PORT = 9090
IP_ADDRESS = "127.0.0.1"

class Network:
    def __init__(self, latency = LATENCY_IN_SECONDS, bandwidth = BANDWIDTH_IN_BYTES, sock = None) -> None:
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
    
    def accept(self) -> tuple:
        return self._socket.accept()