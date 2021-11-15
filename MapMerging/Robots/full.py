from Communications.network import Network, CHUNK_SIZE, IP_ADDRESS, PORT
from Communications.map import Map

class Full_Robot:
    """Transfering FULL map"""

    # REQUIRED
    def __init__(self, name=None, map=None) -> None:
        self.name = name
        self.map = map

    # REQUIRED
    def setup_network(self):
        self.net = Network()

    # REQUIRED
    def get_bytes_sent(self) -> bytes:
        return self.net._bytesSent

    # REQUIRED
    def send_data(self) -> None:
        """ The robot sending information about their local map """
        con, _ = self.net.accept()
        connection = Network(self.net._latency, self.net._bandwidth, sock = con)
        map_data = self.map.to_bytes()
        map_chunks = [map_data[i:i+CHUNK_SIZE] for i in range(0, len(map_data), CHUNK_SIZE)]
        for chunk in map_chunks:
            connection.send(chunk)
        self.net._bytesSent = connection._bytesSent
        connection.close()

    # REQUIRED
    def recv_data(self) -> Map:
        """ The robot who is building the remote copy of a map """
        self.net.connect((IP_ADDRESS, PORT))
        payload = b''
        while True:
            data = self.net.recv(CHUNK_SIZE)
            if data:
                payload += data
            else:
                break
        return self.map.from_bytes(payload)
    
    # REQUIRED
    def close(self) -> None:
        self.net.close()