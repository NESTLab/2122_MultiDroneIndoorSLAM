from __future__ import annotations
from msg.message import Message
from coms.constants import ENCODING
import numpy as np


class Merge(Message):
    id = 0
    map: np.ndarray = None
    dim: str = ""

    def __init__(self: Message, map: np.ndarray = None, dim: str = "") -> None:
        super().__init__()
        self.map = map
        self.dim = dim
        if self.map is not None and self.dim == "":
            self.dim = "{0},{1}".format(self.map.shape[0], self.map.shape[1])

    def produce_payload(self: Merge) -> bytes:
        msg: str = "{0}|{1}|".format(self.id, self.dim)
        return msg.encode(ENCODING) + self.map.tobytes()

    def consume_payload(self: Merge, payload: bytes) -> Message:
        msg = payload.decode(ENCODING)
        parts = msg.split('|')
        id = parts[0]
        if id != str(self.id):
            print("BAD ID: !")
            print(payload)
            return Merge()
        
        raw_dim = parts[1]
        dim = []
        for n in raw_dim.split(','):
            dim.append(int(n, 10))
        
        
        pay = bytearray(payload)
        count = 0
        raw_map = b''
        for i, b in enumerate(pay):
            c = chr(b)
            if c == '|':
                count += 1
                if count == 1:
                    raw_map = pay[i+1:]
        
        map = np.frombuffer(raw_map, dtype=np.uint8)
        
        return Merge(map, raw_dim)

    def handle(self: Message) -> any:
        print(self.map)

    def on_failure() -> None:
        pass
