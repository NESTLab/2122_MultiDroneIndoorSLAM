import numpy as np
from typing import List
from dag import Node, EMPTY_CONTENT

class Map:
    
    def __init__(self, occupancy_grid = np.ndarray([])) -> None:
        self.height = len(occupancy_grid)
        self.width = 0 if self.height == 0 else len(occupancy_grid[0])
        self.dag_store = {}
        self.root = None
    
    def at(self, x:int, y:int) -> int:
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            raise ValueError('Coordinates given to map were out of bounds. Given: x={} y={} when map contains width={} height={}'.format(x, y, self.width, self.height))
        node = self._get_node_at(x, y)
        return node.content

    def _get_node_at(self, x:int, y:int) -> Node:
        x_bounds, y_bounds = (0, self.width), (0, self.height)
        nodes = self._find_nodes_at_helper(x_bounds, y_bounds, x, y, self.root)
        # provide the last node
        return nodes[len(nodes)-1]
    
    def _find_nodes_at_helper(self, x_bounds:tuple, y_bounds:tuple, x:int, y:int, node_str:str) -> List(Node):
        node = self.dag_store[node_str]
        results = [node]
        if not node.empty_content:
            return results
        
        x_start, y_start = x_bounds[0], y_bounds[0]
        x_end, y_end = x_bounds[1], y_bounds[1]
        dist_width, dist_height = x_end - x_start, y_end - y_start
        half_width, half_height = x_start + dist_width//2, y_start + dist_height//2

        # Find which section the coordinate belongs to
        if x < half_width:
            x_b = (x_start, half_width)
            if y < half_height:
                return results + self._find_node_at_helper(x_b, (y_start, half_height), x, y, node.children[0])
            else:
                return results + self._find_node_at_helper(x_b, (half_height, y_end), x, y, node.children[1])
        else:
            x_b = (half_width, x_end)
            if y < half_height:
                return results + self._find_node_at_helper(x_b, (y_start, half_height), x, y, node.children[2])
            else:
                return results + self._find_node_at_helper(x_b, (half_height, y_end), x, y, node.children[3])
    
    # TODO
    def set(self, x:int, y:int, val:int) -> None:
        nodes = self._get_nodes_to_leaf(x, y, val)
        leaf = nodes[len(nodes)-1]
        leaf.content = val
        # Re-hash all nodes from leaf -> root
        old_hash = ""
        prev_updated_node = None
        for node in reversed(nodes) :
            if prev_updated_node is not None:
                # Replace old child hash
                for i in range(0, len(node.children)):
                    if node.children[i] == old_hash:
                        node.children[i] = prev_updated_node.signature
                        break
            
            old_hash = node.hash()
            prev_updated_node = node
            # Add to dictionary
            self.dag_store[node.signature] = node
        # Replace the root
        self.root = prev_updated_node
    
    # TODO
    def _get_nodes_to_leaf(self, x:int, y:int, val:int) -> List[Node]:
        x_bounds, y_bounds = (0, self.width), (0, self.height)
        nodes = self._find_nodes_at_helper(x_bounds, y_bounds, x, y, self.root)
        last:Node = nodes[len(nodes)-1]
        if last.is_leaf() or last.content == val:
            return nodes
        # TODO: Create more nodes until a leaf is present
        return []        

    def occupancy_grid_to_dag(self) -> Node:
        pass

    def dag_to_occupancy_grid(self) -> np.ndarray:
        pass


