#!/usr/bin/env python3.8

import rospy
import numpy as np
import copy

from matplotlib import pyplot, colors
from scipy import signal as sig

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells, MapMetaData, Odometry
from geometry_msgs.msg import Point, PoseStamped
from tf import TransformListener

class publish_inputs_for_learning:

    def __init__(self):
        rospy.init_node('publish_inputs_rl')
        self.robot_namespace = '/tb3_0'
        # self.desired_dimension_x = 640
        # self.desired_dimension_y = 640
        # self.desired_dimension_x = 288
        # self.desired_dimension_y = 160
        self.desired_dimension_x = 352
        self.desired_dimension_y = 224
        self.map = OccupancyGrid()
        self.map_cells = np.array([])
        self.fringe = GridCells()
        self.pub_msg = OccupancyGrid()
        self.odom = Odometry()
        self.resized_map_set = False
        self.pose_map_sent = False
        self.fringe_map_sent = False
        self.full_map_sent = False

        #below commented because size of origional map
        self.mapSub = rospy.Subscriber(self.robot_namespace+'/map', OccupancyGrid, self.map_callback)
        self.mapPub = rospy.Publisher(self.robot_namespace+'/rl_map', OccupancyGrid, queue_size=10)
        self.posePub = rospy.Publisher(self.robot_namespace+'/rl_pose', OccupancyGrid, queue_size=10)
        self.frontierPub = rospy.Publisher(self.robot_namespace+'/rl_frontiers', OccupancyGrid, queue_size=10)
        self.fringeSub = rospy.Subscriber('/map_fringe', GridCells, self.fringe_callback)
        self.poseSub = rospy.Subscriber(self.robot_namespace+'/odom', Odometry, self.odom_callback)

        self.last_map = OccupancyGrid()
        self.last_fringe = GridCells()
        self.last_odom = Odometry()


    def map_callback(self, msg):
        self.map = msg
        map_height = self.map.info.height
        map_width = self.map.info.width
        self.map_cells = np.array(self.map.data)
        resized_map = np.reshape(self.map_cells, (map_height, map_width))
        resized_map = np.pad(resized_map, [(0, self.desired_dimension_y-map_height), (0, self.desired_dimension_x-map_width)], mode='constant', constant_values=-1)
        final_resized_map = resized_map.reshape(-1)

        self.pub_msg = OccupancyGrid()
        pub_msg_info = MapMetaData()
        pub_msg_header = Header()
        pub_msg_info.height = self.desired_dimension_y
        pub_msg_info.width = self.desired_dimension_x
        pub_msg_info.origin = self.map.info.origin
        pub_msg_info.resolution = self.map.info.resolution
        self.pub_msg.info = pub_msg_info
        pub_msg_header.frame_id = self.map.header.frame_id
        self.pub_msg.header = pub_msg_header
        self.pub_msg.data = final_resized_map

        self.resized_map_set = True

        self.mapPub.publish(self.pub_msg)

        # if not self.full_map_sent:
        # rospy.loginfo("FULL MAP SENT")
            # self.full_map_sent = True

        self.last_map = msg



    def fringe_callback(self, msg):
        self.fringe = msg

        if self.resized_map_set:
            cells = msg.cells

            fringe_map = OccupancyGrid()
            fringe_arr = np.full((self.desired_dimension_y, self.desired_dimension_x), -1)
            fringe_resized_map = fringe_arr.reshape(-1)
            index_arr = []
            for c in cells:
                index_arr.append(int(self.point_to_index((c.x, c.y), self.pub_msg)))

            for i in index_arr:
                fringe_resized_map[i] = 0

            int_fringe_resized_map = fringe_resized_map.astype(np.int8)

            fringe_map_info = MapMetaData()
            fringe_map_header = Header()
            fringe_map_info.height = self.desired_dimension_y
            fringe_map_info.width = self.desired_dimension_x
            fringe_map_info.origin = self.map.info.origin
            fringe_map_info.resolution = self.map.info.resolution
            fringe_map.info = fringe_map_info
            fringe_map_header.frame_id = self.map.header.frame_id
            fringe_map.header = fringe_map_header
            fringe_map.data = int_fringe_resized_map
            self.frontierPub.publish(fringe_map)

            # if not self.fringe_map_sent:
            # rospy.loginfo("FRINGE MAP SENT")
                # self.fringe_map_sent = True

        self.last_fringe = msg


    def odom_callback(self, msg):
        self.odom = msg

        if self.resized_map_set:
            odom_map = OccupancyGrid()
            odom_arr = np.full((self.desired_dimension_y, self.desired_dimension_x), -1)
            odom_resized_map = odom_arr.reshape(-1)

            index_of_robot = int(self.point_to_index((msg.pose.pose.position.x, msg.pose.pose.position.y), self.pub_msg))

            odom_resized_map[index_of_robot] = 0

            int_odom_resized_map = odom_resized_map.astype(np.int8)

            odom_map_info = MapMetaData()
            odom_map_header = Header()
            odom_map_info.height = self.desired_dimension_y
            odom_map_info.width = self.desired_dimension_x
            odom_map_info.origin = self.map.info.origin
            odom_map_info.resolution = self.map.info.resolution
            odom_map.info = odom_map_info
            odom_map_header.frame_id = self.map.header.frame_id
            odom_map.header = odom_map_header
            odom_map.data = int_odom_resized_map
            self.posePub.publish(odom_map)

            # if not self.pose_map_sent:
            # rospy.loginfo("POSE MAP SENT")
                # self.pose_map_sent = True

        self.last_odom = msg


    def world_to_map(self, x, y, my_map):
        """
            converts a point from the world to the map
            :param x: float of x position
            :param y: float of y position
            :return: tuple of converted point
        """

        return self.convert_location((x, y), my_map)


    def convert_location(self, loc, my_map):
        """converts points to the grid"""
        res = my_map.info.resolution
        Xorigin = my_map.info.origin.position.x
        Yorigin = my_map.info.origin.position.y

        # print("loc: " + str(loc))
        # print("Xorigin: " + str(Xorigin))
        # print("res: "+ str(res))

        # offset from origin and then divide by resolution
        Xcell = int((loc[0] - Xorigin - (res / 2)) / res)
        Ycell = int((loc[1] - Yorigin - (res / 2)) / res)
        return (Xcell, Ycell)


    def point_to_index(self, point, my_map):
        """convert a index to a point"""
        pt = self.world_to_map(point[0], point[1], my_map)
        return pt[1] * my_map.info.width + pt[0]


    def world_to_map(self, x, y, my_map):
        """
            converts a point from the world to the map
            :param x: float of x position
            :param y: float of y position
            :return: tuple of converted point
        """

        return self.convert_location((x,y), my_map)


if __name__ == '__main__':
    node = publish_inputs_for_learning()

    while not rospy.is_shutdown():
        pass