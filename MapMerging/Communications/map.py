from __future__ import annotations # Allows classes to type hint their own class.
import numpy as np

class Map():
    def __init__(self, map) -> None:
        self.map = map
        self.num_rows = len(map)
        self.num_col = len(map[0])

    def __repr__(self) -> str:
        return str(self.map)
    
    def to_bytes(self) -> bytes:
        return self.map.tobytes()
    
    def from_bytes(self, data) -> Map:
        m = np.frombuffer(data, dtype=np.uint8)
        m = m.reshape(self.num_rows, self.num_col)
        return Map(m)