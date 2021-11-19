import numpy as np
import hashlib
from Communications.map import Map

# Consistant hash function
def uhash(data) -> bytes:
    h = hashlib.sha256()
    h.update(str(data).encode())
    return h.digest()

class Node():
    def __init__(self, content = "", children = []) -> None:
        self.content = str(content)
        self.children = children
    
    def hash(self) -> int:
        for c in self.children:
            self.content += str(c)
        return uhash(self.content)

class DAG():
    def __init__(self, map, resolution=2) -> None:
        self.resolution = resolution
        self.store = {}
        new_map = []
        for row in map:
            r = []
            for content in row:
                address = uhash(content)
                if address not in self.store:
                    self.store[address] = content
                r.append(Node(content=address))
            new_map.append(r)
        self.root = self.build_hierarchy(new_map)
    
    # Repeatedly split the grid into 4ths making the map smaller each iteration
    def build_hierarchy(self, map) -> Node:
        while len(map) > 1 or len(map[0]) > 1:
            height, width = len(map), len(map[0])
            # Create a smaller map
            next_map = []
            for y_start in range(0, height, self.resolution):
                row = []
                for x_start in range(0, width, self.resolution):
                    node = self.compress(map, y_start, x_start)
                    row.append(node)
                
                next_map.append(row)
            map = next_map
        return map[0][0]

    # Creates a node from a section of the map
    def compress(self, map, y_start, x_start) -> Node:
        children = []
        for y in range(y_start, y_start + self.resolution):
            if y < len(map):
                for x in range(x_start, x_start + self.resolution):
                    # Collect all children in this section
                    if x < len(map[y]):
                        content = map[y][x]
                        address = content.hash()
                        if address not in self.store:
                            self.store[address] = content
                        children.append(address)
        return Node(children=children)

    def to_map(self) -> Map:
        pass
