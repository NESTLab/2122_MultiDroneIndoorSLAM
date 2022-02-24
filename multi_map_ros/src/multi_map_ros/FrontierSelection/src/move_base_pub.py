#!/usr/bin/env python3.8

import rospy
import numpy as np
import copy
from scipy import signal as sig

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point, PoseStamped
from tf import TransformListener

class move_base_msg_forwarder:
    def __init__(self):
        rospy.init_node('move_base_msg_forwarder')
        self.robot_namespace = '/tb3_0'
        self.move_helper_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.helper)
        self.move_helper_pub = rospy.Publisher(self.robot_namespace+'/move_base_simple/goal', PoseStamped, queue_size=10)

    def helper(self, msg):
        self.move_helper_pub.publish(msg)


if __name__ == '__main__':
    node = move_base_msg_forwarder()

    while not rospy.is_shutdown():
        pass