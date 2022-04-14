import os
import time

def save_maps(i):
    os.system(f"rosrun map_server map_saver -f mapdata/robot0_map{i} map:=/tb3_0/map")
    os.system(f"rosrun map_server map_saver -f mapdata/robot1_map{i} map:=/tb3_1/map")
    os.system(f"rosrun map_server map_saver -f mapdata/robot2_map{i} map:=/tb3_2/map")  

i = 0
while True:
    save_maps(i)
    i += 1
    time.sleep(10)
    if i >250:
        break
    
