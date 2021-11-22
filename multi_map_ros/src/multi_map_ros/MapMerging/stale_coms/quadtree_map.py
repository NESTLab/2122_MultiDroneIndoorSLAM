from numpy.random.mtrand import rand
from map import Map
import numpy as np
import matplotlib.pyplot as plt

# from MapMerging.src.mapmerge.constants import *

# TODO import constants properly
FREE = 255
OCCUPIED = 0
UNKNOWN = 127

SPLIT_IND = 0
FREE_IND = 1
UNKNOWN_IND = 2
OCCUPIED_IND = 3

IND_MAP = {
    FREE_IND : FREE,
    UNKNOWN_IND : UNKNOWN,
    OCCUPIED_IND : OCCUPIED
}

class QuadTreeMap(Map):
    def __init__(self, map) -> None:
        self.map = map
        self.num_rows = len(map)
        self.num_col = len(map[0])

    def __repr__(self) -> str:
        return str(self.map)

    @staticmethod
    def to_quadtree(map) -> np.array:
        """
        Convert map (2d numpy array) to quadtree representation (1d array)
        where the first two numbers indicate the map size
        and the rest follows the recursive quadtree stream convention:
            0 -- split into quadrants
            1 -- current quadrant is all free
            2 -- current quadrant is all unknown
            3 -- current quadrant is all occupied
        Uses first 8 bytes to transfer map size.
        Uses one extra byte to indicate if array has top/bottom padding (if size is odd)
        """
        def split(arr):
            """
            Split array into four equal quadrants
            """
            mid_x, mid_y = arr.shape[0] // 2, arr.shape[1] // 2
            q1 = arr[0:mid_x, 0:mid_y]
            q2 = arr[0:mid_x, mid_y::]
            q3 = arr[mid_x::, 0:mid_y]
            q4 = arr[mid_x::, mid_y::]
            return [q1, q2, q3, q4]
        
        def encode_size(size, num_bytes_dim=4):
            """
            Not sure if we need this or we can assume fixed size
            Although this makes things more flexible for the SLAM team
            """
            x, y = size
            x_bytes = []
            y_bytes = []
            while x > 255:
                x_bytes.append(255)
                x = x - 255
            while y > 255:
                y_bytes.append(255)
                y = y - 255
            if x > 0:
                x_bytes.append(x)
            if y > 0:
                y_bytes.append(y)
            while len(x_bytes) < num_bytes_dim:
                x_bytes.append(0)
            while len(y_bytes) < num_bytes_dim:
                y_bytes.append(0)
            return x_bytes + y_bytes
        
        def encode_padding(size):
            """
            Encode whether or not we use left | top padding
            """
            x, y = size
            x_pad, y_pad = 0, 0
            if x % 2 != 0:
                x_pad = 1
            if y % 2 != 0:
                y_pad = 1
            return x_pad + 2*y_pad, x_pad, y_pad
            
        # initialize quadtree stream with the data describing the size of the map
        encoded_dims = encode_size(map.shape)
        encoded_pad, x_pad, y_pad = encode_padding(map.shape)
        quadtree_stream = encoded_dims + [encoded_pad]

        # pad left and top with one row of unknown
        padded_map = map
        if x_pad:
            padded_map = np.pad(padded_map, ((1, 0), (0, 0)), mode="constant", constant_values=UNKNOWN)
        if y_pad:
            padded_map = np.pad(padded_map, ((0, 0), (1, 0)), mode="constant", constant_values=UNKNOWN)

        stack = [padded_map]
        while (len(stack) > 0):
            curr = stack.pop(0)
            cells = np.unique(curr)
            if len(cells) > 1:
                stack = split(curr) + stack  # prepend
                quadtree_stream.append(SPLIT_IND)
            elif len(cells) == 0:
                quadtree_stream.append(UNKNOWN_IND)
            else:
                if cells == UNKNOWN:
                    quadtree_stream.append(UNKNOWN_IND)
                elif cells == FREE:
                    quadtree_stream.append(FREE_IND)
                else:
                    quadtree_stream.append(OCCUPIED_IND)
        quadtree_arr = np.asarray(quadtree_stream, dtype=np.uint8)
        return quadtree_arr
    
    @staticmethod
    def decode_quadtree(quadtree_arr) -> np.array:
        """
        Read a quadtree stream and convert it back to a numpy array (map)
        """
        def decode_size(quadtree_arr, num_bytes_dim=4):
            x_bytes = quadtree_arr[:num_bytes_dim]
            y_bytes = quadtree_arr[num_bytes_dim:num_bytes_dim*2]
            x, y = 0, 0
            for v in x_bytes:
                x += v
            for v in y_bytes:
                y += v
            return x, y
        
        def decode_padding(quadtree_arr):
            padding = quadtree_arr[0]
            x_pad, y_pad = 0, 0
            if padding == 1:
                x_pad = 1
            elif padding == 2:
                y_pad = 1
            elif padding == 3:
                x_pad = 1
                y_pad = 1
            return x_pad, y_pad
        
        # take shape from head of quadtree
        shape = decode_size(quadtree_arr)
        quadtree_arr = quadtree_arr[8:] 
        # decode padding if needed (for post processing)
        x_pad, y_pad = decode_padding(quadtree_arr)
        quadtree_arr = quadtree_arr[1:]
        
        # array to load values into
        decoded = np.zeros(shape=(shape[0]+x_pad, shape[1]+y_pad), dtype=np.uint8)

        # keep track of where we will be putting in values
        offset_stack = [(0, 0, shape[0]+x_pad, shape[1]+y_pad)]
        for val in quadtree_arr:
            start_x, start_y, end_x, end_y = offset_stack.pop(0)
            if val != SPLIT_IND:
                decoded[start_x:end_x, start_y:end_y] = IND_MAP[val]
            else:
                # pre-pend new offset indices for four splits
                mid_x, mid_y = (end_x + start_x) // 2, (end_y + start_y) // 2
                q1_coord = (start_x, start_y, mid_x,  mid_y)
                q2_coord = (start_x, mid_y, mid_x, end_y)
                q3_coord = (mid_x, start_y, end_x, mid_y)
                q4_coord = (mid_x, mid_y, end_x, end_y)
                offset_stack = [q1_coord, q2_coord, q3_coord, q4_coord] + offset_stack
        # handle padding
        decoded = decoded[x_pad:, y_pad:,]
        return decoded
        
    def to_bytes(self) -> bytes:
        return QuadTreeMap.to_quadtree(self.map).tobytes()

    def from_bytes(self, data) -> bytes:
        qt = np.frombuffer(data, dtype=np.uint8)
        m = QuadTreeMap.decode_quadtree(qt)
        return Map(m)

if __name__ == "__main__":
    # basic sanity check using random maps, make sure we dont lose any information
    for size in [(128, 128), (200, 300), (201, 201), (201, 200), (200, 201)]:
        rand_map = np.random.uniform(low=0, high=1, size=(size))
        rand_map = np.where((rand_map > 0.33)&(rand_map <= 0.66), UNKNOWN, rand_map)
        rand_map = np.where((rand_map > 0.66)&(rand_map <= 1.00), FREE, rand_map)
        rand_map = np.where(rand_map <= 0.33, OCCUPIED, rand_map)
        rand_map = rand_map.astype(np.uint8)
        qt = QuadTreeMap(rand_map)
        enc = qt.to_bytes()
        dec = qt.from_bytes(enc)
        print(f"Difference between decoded maps (size={size})", np.sum(dec.map != rand_map))
        assert np.sum(dec.map != rand_map) == 0, "maps should not differ"