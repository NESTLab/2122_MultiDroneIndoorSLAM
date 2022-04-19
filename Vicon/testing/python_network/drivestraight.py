#!/usr/bin/env python3

import select
from socket import *
import time
import signal

HOST=''
PORT=22222

def recv_message(s_socket):
    size_length = 4
    size_bytes = b''
    data = b''

    while size_length > 0:
        recv = s.recv(size_length)
        if len(recv) == 0:
            return size_bytes, data
        size_length -= len(recv)
        size_bytes += recv
    size = int.from_bytes(size_bytes, byteorder='big')

    while size > 0:
        recv = s.recv(size)
        size -= len(recv)
        data += recv
    return data, size_bytes

def send_message(s_socket, msg):
    msg = msg.encode() # + b'\x00'
    print(len(msg))
    preamble = (len(msg)).to_bytes(4, byteorder='big') 
    print(preamble)
    print(msg)
    s.send(preamble)
    s.send(msg)

def send_speed(left, right):
    if left < 0:
        left += 255
    
    if right < 0:
        right += 255

    left = left.to_bytes(1, byteorder='big')
    right = right.to_bytes(1, byteorder='big')

    drive_message = b'\x00\x00\x00\x03\x04'
    conn.send(drive_message + left + right)

def signal_handler(signal, frame):
    global run
    if run :
        run = False
    else:
        exit(1)

run = True
with socket(AF_INET, SOCK_STREAM) as s:
    signal.signal(signal.SIGINT, signal_handler)
    # eventually allow for more than one robot
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    s.close()

    while run:
        send_speed(60,60)
        time.sleep(50)

    conn.send(b'\x00\x00\x00\x01\x05') # stop
    conn.close()
