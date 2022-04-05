import trio
import numpy as np
from rospy import Publisher
from typing import Tuple
from coms.utils import debug, get_map, add_padding, map_to_chunks, decompress_map, pack_bytes, read_id_chunk, gen_id_chunk, read_all_chunks
from coms.constants import MAP_MSG_ID, PING_MSG_ID, CHUNK_SIZE, NEXT_MEETING_MSG_ID, READY_TO_MEET_ID
from coms.messages import gen_sync_msg_chunks, read_sync_msg_data, gen_next_meeting_message, read_next_meeting_message, gen_ready_to_meet_chunks, read_ready_to_meet_data
from itertools import count
from typing import List

class Client():
    LOCAL_IP = "127.0.0.1"
    DEBUGGER: Publisher = None
    def __init__(self, ip: str, debug_pub: Publisher, namespace: str) -> None:
        self.LOCAL_IP = ip
        self.DEBUGGER = debug_pub
        self.namespace = namespace
    
    async def ready_to_meet(self, ip: str, port: int, status: bool) -> bool:
        chunks = gen_ready_to_meet_chunks(status)
        try:
            msg_id = -1
            raw_resp: bytes = b''
            client_stream = await trio.open_tcp_stream(host=ip, port=port, local_address=self.LOCAL_IP)
            async with client_stream:
                # write
                for chunk in chunks:
                    await client_stream.send_all(chunk)
                await client_stream.send_eof()
                # read
                msg_id, raw_resp = await read_all_chunks(client_stream)

            if msg_id != READY_TO_MEET_ID:
                debug(self.DEBUGGER, f"Client {self.LOCAL_IP} encountered unexpected message id {msg_id} from {ip} [ERROR]")
                return False

            if raw_resp == b'':
                return False

            ready_to_meet_msg: dict = await read_ready_to_meet_data(raw_resp)
            resp_status = bool(ready_to_meet_msg['status'])
            s = 'READY'
            if not resp_status:
                s = 'NOT READY'
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} retrieved a {s} reply from {ip} [READY_TO_MEET]")
            return bool(resp_status)

        except Exception as e:
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} closed connection with {ip} [ERROR] {e}")
            return False    

    async def next_meeting(self, ip: str, port: int, point: Tuple[int, int, int], time_to_meet: float) -> bool:
        chunks: List[bytes] = gen_next_meeting_message(point, time_to_meet)
        try:
            msg_id = -1
            raw_resp: bytes = b''
            client_stream = await trio.open_tcp_stream(host=ip, port=port, local_address=self.LOCAL_IP)
            async with client_stream:
                # write
                for chunk in chunks:
                    await client_stream.send_all(chunk)
                await client_stream.send_eof()
                # read
                msg_id, raw_resp = await read_all_chunks(client_stream)

            if msg_id != NEXT_MEETING_MSG_ID:
                debug(self.DEBUGGER, f"Client {self.LOCAL_IP} encountered unexpected message id {msg_id} from {ip} [ERROR]")
                return False

            if raw_resp == b'':
                return False

            msg: dict = await read_next_meeting_message(raw_resp)
            accepted = msg['accepted']
            s = 'DENY'
            if accepted:
                s = 'ACCEPT'
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} saw robot {ip} {s} the next meeting [NEXT_MEETING]")
            return accepted

        except Exception as e:
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} closed connection with {ip} [ERROR] {e}")
            return False
   

    # Send and recieve map data
    async def sync(self, ip: str, port: int, role: str) -> bool:
        local_map: np.ndarray = get_map(self.namespace)
        chunks: List[bytes] = gen_sync_msg_chunks(local_map, role)
        try:
            msg_id = -1
            raw_resp: bytes = b''
            client_stream = await trio.open_tcp_stream(host=ip, port=port, local_address=self.LOCAL_IP)
            async with client_stream:
                # write
                for chunk in chunks:
                    await client_stream.send_all(chunk)
                await client_stream.send_eof()
                # read
                msg_id, raw_resp = await read_all_chunks(client_stream)

            if msg_id != MAP_MSG_ID:
                debug(self.DEBUGGER, f"Client {self.LOCAL_IP} encountered unexpected message id {msg_id} from {ip} [ERROR]")
                return False

            if raw_resp == b'':
                return False

            map_msg: dict = await read_sync_msg_data(raw_resp)
            # TODO: Call map merge service
            map = map_msg['map']
            role = map_msg['role']
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} retrieved sync from robot with role: {role} [ROLE]")
            return (map == local_map).all

        except Exception as e:
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} closed connection with {ip} [ERROR] {e}")
            return False

    # Send and recive a 'ping' message
    async def ping(self, ip: str, port: int, attempts: int = 1) -> bool:
        if attempts == 0:
            return False
        try:
            msg_id = -1
            raw_resp: bytes = b''
            client_stream = await trio.open_tcp_stream(host=ip, port=port, local_address=self.LOCAL_IP)
            async with client_stream:
                # write
                await client_stream.send_all(gen_id_chunk(PING_MSG_ID))
                await client_stream.send_all(b"ping")
                await client_stream.send_eof()
                # read
                msg_id, raw_resp = await read_all_chunks(client_stream)

            return (msg_id == PING_MSG_ID and raw_resp == b'ping') 
        except Exception as e:
            debug(self.DEBUGGER, f"Client {self.LOCAL_IP} closed connection with {ip} [ERROR] {e}")
            return await self.ping(ip, port, attempts-1)

class Server():
    CONNECTION_COUNTER: count
    LOCAL_IP = "127.0.0.1"
    LOCAL_PORT = "8887"

    def __init__(self, ip: str, port: int, debug_pub: Publisher, namespace: str, role: str) -> None:
        self.LOCAL_IP = ip
        self.LOCAL_PORT = port
        self.CONNECTION_COUNTER = count()
        self.DEBUGGER = debug_pub
        self.namespace = namespace
        self.role = role

    async def handler(self, server_stream):
        # Assign each connection a unique number to make our debug prints easier
        # to understand when there are multiple simultaneous connections.
        ident = next(self.CONNECTION_COUNTER)
        client_ip, _ = server_stream.socket.getpeername()
        debug(self.DEBUGGER, f"Server {self.LOCAL_IP} connected to {client_ip} [ID #{ident} OPEN]")
        msg_id = -1
        raw_bytes: bytes = b''
        try:
            # read
            msg_id, raw_bytes = await read_all_chunks(server_stream)

            # write
            chunks = []
            if msg_id == MAP_MSG_ID:
                map_msg: dict = await read_sync_msg_data(raw_bytes)
                map = map_msg['map']
                role = map_msg['role']
                chunks = gen_sync_msg_chunks(map, self.role)
                debug(self.DEBUGGER, f"Server {self.LOCAL_IP} got SYNC from {client_ip} with role {role} [ID #{ident} SYNC]")
            elif msg_id == PING_MSG_ID:
                chunks = [gen_id_chunk(PING_MSG_ID), b'ping']
                debug(self.DEBUGGER, f"Server {self.LOCAL_IP} got PING from {client_ip} [ID #{ident} PING]")
            elif msg_id == NEXT_MEETING_MSG_ID:
                next_meeting: dict = await read_next_meeting_message(raw_bytes)
                point = next_meeting['point']
                time_to_meet = next_meeting['time_to_meet']
                chunks = [gen_id_chunk(NEXT_MEETING_MSG_ID), add_padding('true')]
                debug(self.DEBUGGER, f"Server {self.LOCAL_IP} got NEXT_MEETING from {client_ip} with point: {point} [ID #{ident} NEXT_MEETING]")
            elif msg_id == READY_TO_MEET_ID:
                ready_to_meet_msg: dict = await read_ready_to_meet_data(raw_bytes)
                client_status = ready_to_meet_msg['status']
                s = 'READY'
                if not client_status:
                    s = 'NOT READY'
                # TODO: Change to current status
                chunks = gen_ready_to_meet_chunks(client_status)
                debug(self.DEBUGGER, f"Server {self.LOCAL_IP} got {s} from {client_ip} [ID #{ident} READY_TO_MEET]")
            else:
                debug(self.DEBUGGER, f"Server {self.LOCAL_IP} encountered unexpected message id {msg_id} from {client_ip} [ID #{ident} ERROR]")
            
            for chunk in chunks:
                await server_stream.send_all(chunk)
            await server_stream.send_eof()
            debug(self.DEBUGGER, f"Server {self.LOCAL_IP} closed connection with {client_ip} [ID #{ident} CLOSED]")
        except Exception as e:
            debug(self.DEBUGGER, f"Server {self.LOCAL_IP} closed connection with {client_ip} [ID #{ident} ERROR] {e}")
        
    
    async def serve(self):
        print(f"Serving -> {self.LOCAL_IP}:{self.LOCAL_PORT}")
        await trio.serve_tcp(self.handler, self.LOCAL_PORT, host=self.LOCAL_IP)
        print(f"Stopped Serving <- {self.LOCAL_IP}:{self.LOCAL_PORT}")

# EXAMPLES
    
# Run a server
# s = Server("192.168.0.2", 12345)
# trio.run(s.serve)

# Run a client
# c = Client("192.168.0.1")
# trio.run(c.sync, "192.168.0.2", 12345)
