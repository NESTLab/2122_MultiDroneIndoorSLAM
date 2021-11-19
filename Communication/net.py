import time
from utils import b_to_mb
from socket import socket
from poorconn import delay_before_sending, delay_before_sending_upon_acceptance, make_socket_patchable

PACKET_SIZE:bytes = 1024
LATENCY:int = 20                                    # Mbps
PACKET_DELAY:float = PACKET_SIZE / b_to_mb(LATENCY) # Seconds

def _client_socket() -> socket:
    s = socket()
    s = make_socket_patchable(s)
    delay_before_sending(s, PACKET_DELAY, PACKET_SIZE)
    return s

def _server_socket() -> socket:
    s = socket()
    s = make_socket_patchable(s)
    delay_before_sending_upon_acceptance(s, PACKET_DELAY, PACKET_SIZE)
    return s
