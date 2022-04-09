#!/usr/bin/env python3

import select
from socket import *
import struct
import argparse

HOST='127.0.0.1'
PORT='22222'

def send_message(s_socket, msg):
    # msg = msg.encode() + b'\x00'
    print(len(msg))
    preamble =(len(msg)).to_bytes(4, byteorder='big') 
    print(preamble)
    print(msg)
    s.send(preamble)
    s.send(msg)

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

parser = argparse.ArgumentParser(description="Parse networking arguments");
parser.add_argument('-n', '--name', type=str, default='Khepera_1', help="Name of the fake robot");
parser.add_argument('-ip', type=str, default='127.0.0.1', help='IP of argos server.')
parser.add_argument('-p', '--port', type=str, default='22222', help='Port of argos server.')
args= parser.parse_args()

if __name__=="__main__":
    s = create_connection((args.ip, args.port))

    print('Connected')

    while True:
        data, preamble = recv_message(s)
        print('Preamble: {}, Data: {}'.format(preamble, data))
        if data == b'\x01' or data+preamble == b'':
            print('Disconnecting')
            break;
        elif len(data) < 1:
            continue 
        
        command = int.from_bytes([data[0]], byteorder='big')
        data = data[1:]
        print(command)
        if command == 0:
            print('Name Request')
            send_message(s, args.name.encode() + b'\x00')
            # print('name sent')
        elif command == 2:
            speed = int.from_bytes(data, byteorder='big')
            print('Right Wheel Speed: {}'.format(speed))
        elif command == 3:
            speed = int.from_bytes(data, byteorder='big')
            print('Left Wheel Speed: {}'.format(speed))
        elif command == 4:
            print(data)
            l_speed = int.from_bytes([data[0]], byteorder='big')
            r_speed = int.from_bytes([data[1]], byteorder='big')
            if l_speed > 127:
                l_speed -= 255
            if r_speed > 127:
                r_speed -= 255
            print("Driving at ({}/127,{}/127)".format(l_speed, r_speed))
        elif command == 5:
            print("Stop Driving")
        send_message(s, b'\x02\x0f\x0f')

