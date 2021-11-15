from Communications.network import Network, CHUNK_SIZE, IP_ADDRESS, PORT
from Communications.map import Map
from Communications.dag import DAG

class DAG_Robot():
    """ Transfering partial DAG of map """

    # REQUIRED
    def __init__(self, name=None, map=None) -> None:
        self.name = name
        self.map = map
        if map != None:
            self.dag = DAG(map.map)
        # To be recieved/built within recv_data
        self.remote_map = None

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
        self.remote_map = self.map.from_bytes(payload)
        return self.remote_map

    # REQUIRED
    def close(self) -> None:
        self.net.close()

    def gen_neighbor_map(self, con) -> Map:
        r = self.dag.root.hash()

        # TODO:
        # deep copy the existing map
        # find the ONE node that doesnt match
        # keep iterating down from there
        # IF at any time, we can resolve a hash, just do it!
        # OBJECTIVE: update the hash map (store) with new configuration of 0 and 1s
        # After building the neighbor DAG, call dag.to_map()