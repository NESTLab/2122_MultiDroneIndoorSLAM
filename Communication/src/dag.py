from Communication.src.utils import uhash

# NOTE: This cannot conflict with the values of the occupancy grid
EMPTY_CONTENT = -38

class Node:
    
    def __init__(self) -> None:
        self.children = []
        self.content = EMPTY_CONTENT
        self.signature = ""
    
    def __str__(self) -> str:
        s = "{}:".format(str(self.content))
        s += ":".join(self.children)
        return s

    def hash(self) -> str:
        self.signature = uhash(str(self))
        return self.signature

    def empty_content(self) -> bool:
        return self.content == EMPTY_CONTENT
    
    def is_leaf(self) -> bool:
        return len(self.children) == 0 and self.content != EMPTY_CONTENT
