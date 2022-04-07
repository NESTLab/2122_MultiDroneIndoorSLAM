from typing import List, Tuple
from coms.utils import map_to_chunks, read_all_chunks, decompress_map, gen_id_chunk, add_padding
from coms.constants import CHUNK_SIZE, ENCODING, PADDING_CHAR, NEXT_MEETING_MSG_ID, READY_TO_MEET_ID, INFO_MSG_ID
from trio import SocketStream
import numpy as np

# Format: (REQUEST & RESPONSE) id_chunk, role_chunk, map_chunks
def gen_sync_msg_chunks(map: np.ndarray, role:str) -> List[bytes]:
    return map_to_chunks(map, role = role)

def read_sync_msg_data(data: bytes) -> dict:
    role_chunk = (data[:CHUNK_SIZE]).decode(ENCODING)
    role = role_chunk.replace(PADDING_CHAR, '')
    map = decompress_map(data[CHUNK_SIZE:])
    return { 'role': role, 'map': map }

def gen_next_meeting_message(point: Tuple[int, int, int], time_to_meet: float) -> List[bytes]:
    id_chunk = gen_id_chunk(NEXT_MEETING_MSG_ID)
    info_block = add_padding(f"{point[0]}|{point[1]}|{point[2]}|{time_to_meet}")
    return [id_chunk, info_block]

# Format: (REQUEST) id_chunk, point_and_time_to_meet_chunk
#         (RESPONSE) id_chunk, is_accepted_chunk
def read_next_meeting_message(data: bytes) -> dict:
    block_data = data.decode(ENCODING).replace(PADDING_CHAR, '')
    parts = block_data.split('|')
    if len(parts) == 1:
        # This is a response packet
        return { 'accepted': 'true' == parts[0].lower() }
    if len(parts) != 4:
        return { 'accepted': False }

    point = (int(parts[0], 10), int(parts[1], 10), int(parts[2], 10))
    time_to_meet = float(parts[3])
    return { 'point': point, 'time_to_meet': time_to_meet, 'accepted': False }

# Format: (REQUEST & RESPONSE) id_chunk, status_chunk
def gen_ready_to_meet_chunks(ready: bool) -> List[bytes]:
    id_chunk: bytes = gen_id_chunk(READY_TO_MEET_ID)
    status_chunk: bytes = add_padding(str(ready))
    return [id_chunk, status_chunk]

def read_ready_to_meet_data(data: bytes) -> dict:
    status_chunk = (data).decode(ENCODING)
    status = status_chunk.replace(PADDING_CHAR, '')
    status = status.lower() == 'true'
    return { 'status': status }

# Format: (REQUEST & RESPONSE) id_chunk, info_chunk 
def gen_info_msg_chunks(state: str, address: str, parent: str, child: str, role: str) -> List[bytes]:
    id_chunk: bytes = gen_id_chunk(INFO_MSG_ID)
    info_chunk: bytes = add_padding(f"{state}|{address}|{parent}|{child}|{role}")
    return [id_chunk, info_chunk]

def read_info_msg_data(data: bytes) -> dict:
    info_chunk = (data).decode(ENCODING)
    info_data = info_chunk.replace(PADDING_CHAR, '')
    info_parts = info_data.split('|')
    if len(info_parts) != 5:
        return { 'state': '', 'address': '', 'parent': '', 'child': '', 'role': '' }
    return {
        'state': info_parts[0],
        'address': info_parts[1],
        'parent': info_parts[2],
        'child': info_parts[3],
        'role': info_parts[4]
    }

