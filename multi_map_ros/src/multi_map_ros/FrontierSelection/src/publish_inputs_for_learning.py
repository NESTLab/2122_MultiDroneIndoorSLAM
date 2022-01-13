#!/usr/bin/env python3.8

import rospy
import numpy as np
import copy

from matplotlib import pyplot, colors
from scipy import signal as sig

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells, MapMetaData
from geometry_msgs.msg import Point, PoseStamped
from tf import TransformListener

class publish_inputs_for_learning:

    def __init__(self):
        rospy.init_node('find_fontier')
        self.robot_namespace = '/tb3_0'
        self.mapSub = rospy.Subscriber(self.robot_namespace+'/map', OccupancyGrid, self.map_callback)
        self.mapPub = rospy.Publisher(self.robot_namespace+'/rl_map', OccupancyGrid, queue_size=10)
        self.posePub = rospy.Publisher(self.robot_namespace+'/rl_pose', OccupancyGrid, queue_size=10)
        self.frontierPub = rospy.Publisher(self.robot_namespace+'/rl_frontiers', OccupancyGrid, queue_size=10)
        self.map = OccupancyGrid()
        self.map_cells = np.array([])
        self.desired_dimension = 640

    def map_callback(self, msg):
        self.map = msg
        map_height = self.map.info.height
        map_width = self.map.info.width
        self.map_cells = np.array(self.map.data)
        resized_map = np.reshape(self.map_cells, (map_height, map_width))
        resized_map = np.pad(resized_map, [(0, self.desired_dimension-map_height), (0, self.desired_dimension-map_width)], mode='constant')
        final_resized_map = resized_map.reshape(-1)

        pub_msg = OccupancyGrid()
        pub_msg_info = MapMetaData()
        pub_msg_header = Header()
        pub_msg_info.height = self.desired_dimension
        pub_msg_info.width = self.desired_dimension
        pub_msg_info.origin = self.map.info.origin
        pub_msg_info.resolution = self.map.info.resolution
        pub_msg.info = pub_msg_info
        pub_msg_header.frame_id = self.map.header.frame_id
        pub_msg.header = pub_msg_header
        pub_msg.data = final_resized_map
        self.mapPub.publish(pub_msg)




if __name__ == '__main__':
    node = publish_inputs_for_learning()

    while not rospy.is_shutdown():
        pass