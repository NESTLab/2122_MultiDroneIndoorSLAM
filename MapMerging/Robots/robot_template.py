from Communications.network import Network, IP_ADDRESS, PORT
from Communications.map import Map
import numpy as np

class Template:
    """ Only transferring different positions """

    # REQUIRED
    def __init__(self, name=None, map=None) -> None:
        self.name = name
        self.map = map
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

        # TODO:
        # communicate with the other robot


        connection.close()

    # REQUIRED
    def recv_data(self) -> Map:
        """ The robot who is building the remote copy of a map """
        self.net.connect((IP_ADDRESS, PORT))

        peer_map = Map(np.array([]))

        # TODO:
        # communicate with the other robot & construct the peer's map

        self.remote_map = peer_map
        return self.remote_map
    
    # REQUIRED
    def close(self) -> None:
        self.net.close()