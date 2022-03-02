#!/usr/bin/env python3.8

import rospy
import roslaunch
import numpy as np
import copy
from scipy import signal as sig
import subprocess
import signal

from std_msgs.msg import Header, String
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point, PoseStamped
from tf import TransformListener

class move_base_reset:

    def __init__(self):
        rospy.init_node('mapping_reset_node')
        self.robot_namespace = '/tb3_0'
        self.stop_signal = rospy.Subscriber('/stop_move_base', String, self.stop)
        self.start_signal = rospy.Subscriber('/start_move_base', String, self.start)


    def stop(self, msg):
        # self.launch.shutdown()
        print("STOPPING")
        self.child.send_signal(signal.SIGINT)


    def start(self, msg):
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
        #     "/home/taylor/multidrone_slam/src/2122_MultiDroneIndoorSLAM/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_move_base.launch"])
        # self.launch.start()
        # rospy.loginfo("move base started")

        print("STARTING")
        self.child = subprocess.Popen(["roslaunch", "turtlebot3_gazebo", "multi_move_base.launch"])
        # child.wait() #You can use this line to block the parent process until the child process finished.
        # print("parent process")
        # print(self.child.poll())

        rospy.loginfo('The PID of child: %d', self.child.pid)
        # print("The PID of child:", self.child.pid)


if __name__ == '__main__':
    node = move_base_reset()

    while not rospy.is_shutdown():
        pass