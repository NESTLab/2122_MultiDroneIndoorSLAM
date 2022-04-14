import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt

RESOLUTION = 0.05
X, Y = 60, 25

if __name__ == "__main__":
    map_x, map_y = int(X / RESOLUTION), int(Y / RESOLUTION)
    map_size = (map_x, map_y)
    occupancy_grid = np.ones(map_size) * 255
    xmlTree = ET.parse('/home/connor/catkin_ws/src/turtlebot3_gazebo/worlds/dans_monstor_maze.world').getroot()
    models = xmlTree.findall(".//model")
    for model in models:
        children = [c for c in model]
        links = filter(lambda c: c.tag == "link", children)
        for l in links:
            name = l.attrib["name"]
            if "Wall" in name:
                children = [c for c in l]
                collision = [c for c in children if c.tag == "collision"]
                if len(collision) > 0:
                    collision = collision[0]
                    name 
                else:
                    break
                collision_geo = [c for c in collision if c.tag == "geometry"][0]
                collision_box = [c for c in collision_geo if c.tag == "box"][0]
                collision_size = [c for c in collision_box if c.tag == "size"][0]
                collision_pose = [c for c in children if c.tag == "pose"][0]
                size = [int(float(dim)/RESOLUTION) for dim in collision_size.text.split(" ")[:2]]
                pose = [int(float(dim)/RESOLUTION) for dim in collision_pose.text.split(" ")[:2]]
                orientation = float(collision_pose.text.split(" ")[-1])
                if np.abs(orientation) < 3 and np.abs(orientation) > 1.5:
                    size.reverse()
                for i in range(len(pose)):
                    pose[i] += (map_size[i] // 2)
                print(size, pose)
                min_x, max_x = pose[0] - (size[0] // 2), pose[0] + (size[0] // 2)
                min_y, max_y = pose[1] - (size[1] // 2), pose[1] + (size[1] // 2)
                for x in range(min_x, max_x):
                    occupancy_grid[x, min_y:max_y] = 0
    occupancy_grid = occupancy_grid.astype(np.uint8)
    np.savetxt("PARSED_MONSTER.txt", np.flipud(occupancy_grid.T))
    plt.imshow(np.flipud(occupancy_grid.T), cmap="gray")
    plt.show()

    # tags = {elem.tag for elem in xmlTree.iter()}
    # print(tags)