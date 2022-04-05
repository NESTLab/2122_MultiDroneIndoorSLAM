from typing import List
from coms.utils import map_to_chunks, read_all_chunks, decompress_map
from coms.constants import CHUNK_SIZE, ENCODING, PADDING_CHAR
from trio import SocketStream
import numpy as np

# Format: id_chunk, role_chunk, map_chunks
def gen_sync_msg_chunks(map: np.ndarray, role:str) -> List[bytes]:
    return map_to_chunks(map, role = role)

async def read_sync_msg_data(data: bytes) -> dict:
    role_chunk = (data[:CHUNK_SIZE]).decode(ENCODING)
    role = role_chunk.replace(PADDING_CHAR, '')
    map = decompress_map(data[CHUNK_SIZE:])
    return { 'role': role, 'map': map }

