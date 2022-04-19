#!/usr/bin/env python3

import select
from socket import *
import time

HOST=''
PORT=22222

def recv_message(s_socket):
    size_length = 4
    size_bytes = b''
    data = b''

    while size_length > 0:
        recv = s_socket.recv(size_length)
        if len(recv) == 0:
            return data, size_bytes
        size_length -= len(recv)
        size_bytes += recv
    size = int.from_bytes(size_bytes, byteorder='big')
    # print("Message Size {}".format(size))

    while size > 0:
        recv = s_socket.recv(size)
        size -= len(recv)
        data += recv
    return data, size_bytes

def send_message(s_socket, msg):
    try:
        msg = msg.encode() # + b'\x00'
    except:
        pass

    print(len(msg))
    preamble = (len(msg)).to_bytes(4, byteorder='big') 
    # print(preamble)
    # print(msg)
    s_socket.send(preamble)
    s_socket.send(msg)

def send_loop_drive(conn):
    drive_message = b'\x00\x00\x00\x03\x04'
    speed_range = [x for x in range(1,127)] 
    for x in speed_range:
        speed = x.to_bytes(1, byteorder='big')
        conn.send(drive_message + (speed*2))
        time.sleep(.05)

    for x in reversed(speed_range):
        speed = x.to_bytes(1, byteorder='big')
        conn.send(drive_message + (speed*2))
        time.sleep(.05)
    
    for x in speed_range:
        speed = (256-x).to_bytes(1, byteorder='big')
        conn.send(drive_message + (speed*2))
        time.sleep(.05)

    for x in reversed(speed_range):
        speed = (256-x).to_bytes(1, byteorder='big')
        conn.send(drive_message + (speed*2))
        time.sleep(.05)
    conn.send(b'\x00\x00\x00\x01\x05') # stop

def request_sensor_value(conn):
    send_message(conn, b'\x06\xFF\xF0')
    data, __= recv_message(conn)
    print(data)
    send_message(conn, b'\x01')


with socket(AF_INET, SOCK_STREAM) as s:
    # eventually allow for more than one robot
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    s.close()
    try:
        with conn:
            # print('Connected by', addr)
            # send_message(s, b'\x00')
            # data, __ = recv_message()
            # print('Their name is ', data.decode())
            # 
            
            # send_loop_drive(conn);
            #
            request_sensor_value(conn)
    except Exception as e:
        print(e)

    conn.close()
