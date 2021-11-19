import numpy as np
from typing import List
from Communication.src.dag import Node, EMPTY_CONTENT

class Map:
    
    def __init__(self, occupancy_grid = np.ndarray([])) -> None:
        self.height = len(occupancy_grid)
        self.width = 0 if self.height == 0 else len(occupancy_grid[0])
        self.dag_store = {}
        self.root = None    # root hash : string
    
    def at(self, x:int, y:int) -> int:
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            raise ValueError('Coordinates given to map were out of bounds. Given: x={} y={} when map contains width={} height={}'.format(x, y, self.width, self.height))
        node = self._get_node_at(x, y)
        return node.content

    def _get_node_at(self, x:int, y:int) -> Node:
        x_bounds, y_bounds = (0, self.width), (0, self.height)
        
        def find_node(node_str:str) -> Node:
            node = self.dag_store[node_str]
            if not node.empty_content:
                return node
            
            x_start, y_start = x_bounds[0], y_bounds[0]
            x_end, y_end = x_bounds[1], y_bounds[1]
            dist_width, dist_height = x_end - x_start, y_end - y_start
            half_width, half_height = x_start + dist_width//2, y_start + dist_height//2

            # Find which section the coordinate belongs to
            if x < half_width:
                x_b = (x_start, half_width)
                if y < half_height:
                    return find_node(node.children[0])
                else:
                    return find_node(node.children[1])
            else:
                x_b = (half_width, x_end)
                if y < half_height:
                    return find_node(node.children[2])
                else:
                    return find_node(node.children[3])
            
        return find_node(self.root)
    
    def set(self, x:int, y:int, val:int) -> None:
        x_bounds, y_bounds = (0, self.width), (0, self.height)
        
        def find_nodes(node_str:str) -> List[Node]:
            node:Node = self.dag_store[node_str]                
            res = [node]
            if node.content == val or node.is_leaf():
                res
            
            x_start, y_start = x_bounds[0], y_bounds[0]
            x_end, y_end = x_bounds[1], y_bounds[1]
            dist_width, dist_height = x_end - x_start, y_end - y_start
            half_width, half_height = x_start + dist_width//2, y_start + dist_height//2

            # Find which section the coordinate belongs to
            if x < half_width:
                x_b = (x_start, half_width)
                if y < half_height:
                    return res + find_nodes(node.children[0])
                else:
                    return res + find_nodes(node.children[1])
            else:
                x_b = (half_width, x_end)
                if y < half_height:
                    return res + find_nodes(node.children[2])
                else:
                    return res + find_nodes(node.children[3])
                
        nodes:List[Node] = find_nodes(self.root)
        last:Node = nodes[len(nodes)-1]
        
        if last.is_leaf():
            last.content = val
            self.root = self._rehash_nodes(nodes)
            return
        elif last.content != val:
            raise ValueError('Within Set(), find_nodes(root) retrieved a NON leaf node that did not have content similar to the value.')
    
    def _rehash_nodes(self, nodes:List[Node], delete:bool=False) -> str:
        prev_node:Node = nodes[len(nodes)-1]
        prev_sig:str = prev_node.signature
        if delete:
            self.dag_store.pop(prev_sig)
        prev_node.hash()
        self.dag_store[prev_node.signature] = prev_node
        # re-sign nodes in reverse order (leaf to root)
        for n_idx in range(len(nodes) - 2, -1, -1):    
            cur_node:Node = nodes[n_idx]
            for c_idx in range(len(cur_node.children)):
                # Replace child hash
                if cur_node.children[c_idx] == prev_sig:
                    cur_node.children[c_idx] = prev_node.signature
            # Recheck content
            self._reset_node_content(cur_node)
            # Init for next iteration
            prev_sig = cur_node.signature
            cur_node.hash()
            self.dag_store[cur_node.signature] = cur_node
            prev_node = cur_node
            if delete:
                self.dag_store.pop(prev_sig)
        
        return prev_node.signature
    
    def _reset_node_content(self, node:Node) -> Node:
        content_seen = {}
        for c in node.children:
            child:Node = self.dag_store[c]
            content_seen[child.content] = 1
        if len(content_seen.keys()) == 1:
            node.content = list(content_seen.keys())[0]
        else:
            node.content = EMPTY_CONTENT
        return node
    
    def occupancy_grid_to_dag(self) -> Node:
        pass

    def dag_to_occupancy_grid(self) -> np.ndarray:
        pass

class OccupancyGrid:
    def __init__(self, array:np.ndarray, unknown = 0, free = 1, occupied = 2) -> None:
        self.array = array
        