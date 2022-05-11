#!/usr/bin/env python3.8
import math

import rospy
import numpy as np
import copy
from scipy import signal as sig

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from geometry_msgs.msg import Point, PoseStamped
from tf import TransformListener

import matplotlib.pyplot as plt

class exploration_eval:
    def __init__(self):
        rospy.init_node('exploration_eval')
        self.robot_namespace = '/tb3_0'
        self.mapSub = rospy.Subscriber(self.robot_namespace+'/map', OccupancyGrid, self.map_callback)
        self.odomSub = rospy.Subscriber(self.robot_namespace+'/odom', Odometry, self.odom_callback)
        # self.odomPub = rospy.Publisher('/distance_travelled', Odometry, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)

        self.start_time = rospy.get_time() #rospy.get_rostime()

        self.total_distance = 0.0
        self.previous_x = 0
        self.previous_y = 0
        self.first_run = True

        self.time_record = []
        self.percent_of_map_covered_record = []
        self.total_distance_travelled_record = []
        self.percent_of_map_covered_time = []
        self.total_distance_travelled_time = []

        self.map = OccupancyGrid()
        self.odom = Odometry()

        self.files_written = False


    def map_callback(self, msg):

        self.map = msg

        # data = np.asarray(msg.data)
        # values = [0, 101]
        # freq = np.histogram(data, bins=[-np.inf] + values + [np.inf])[0] / data.size
        # # print(freq)
        #
        # # uniques, counts = np.unique(data, return_counts=True)
        # # percentages = dict(zip(uniques, counts / len(data)))
        # # print(percentages)
        #
        # # total_percent_explored = percentages.get(1) + percentages.get(0)
        #
        # total_percent_explored = freq[1]
        #
        # print("total Percent explored: " + str(total_percent_explored))
        #
        # self.percent_of_map_covered_record.append(total_percent_explored)
        # self.percent_of_map_covered_time.append(rospy.get_time() - self.start_time)


    def odom_callback(self, msg):

        self.odom = msg

        # if (self.first_run):
        #     self.previous_x = msg.pose.pose.position.x
        #     self.previous_y = msg.pose.pose.position.y
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        # print(msg)
        # d_increment = math.sqrt(((x - self.previous_x) * (x - self.previous_x)) + ((y - self.previous_y) * (y - self.previous_y)))
        # self.total_distance = self.total_distance + d_increment
        # print("Total distance traveled is: " + str(self.total_distance))
        # # self.odomPub.publish(data)
        # self.previous_x = msg.pose.pose.position.x
        # self.previous_y = msg.pose.pose.position.y
        # self.first_run = False
        #
        # self.total_distance_travelled_record.append(self.total_distance)
        # self.total_distance_travelled_time.append(rospy.get_time() - self.start_time)


    def timer_callback(self, timer_info):

        data = np.asarray(self.map.data)
        odom = self.odom

        values = [0, 101]
        freq = np.histogram(data, bins=[-np.inf] + values + [np.inf])[0] / data.size

        # print(freq)
        #
        # uniques, counts = np.unique(data, return_counts=True)
        # percentages = dict(zip(uniques, counts / len(data)))
        # print(percentages)

        total_percent_explored = freq[1]*100

        print("Total Percent explored: " + str(total_percent_explored))

        self.percent_of_map_covered_record.append(total_percent_explored)

        if (self.first_run):
            self.start_time = rospy.get_time()
            self.previous_x = odom.pose.pose.position.x
            self.previous_y = odom.pose.pose.position.y
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        d_increment = math.sqrt(((x - self.previous_x) * (x - self.previous_x)) + ((y - self.previous_y) * (y - self.previous_y)))
        self.total_distance = self.total_distance + d_increment
        print("Total distance traveled is: " + str(self.total_distance))
        self.previous_x = odom.pose.pose.position.x
        self.previous_y = odom.pose.pose.position.y
        self.first_run = False

        self.total_distance_travelled_record.append(self.total_distance)

        time_diff = rospy.get_time() - self.start_time
        print("Time: " + str(time_diff))
        print("----------------------------------------------------------")
        self.time_record.append(time_diff)

        if time_diff > 1200 and not self.files_written:
            rospy.loginfo("Writing Eval Graphs to Files")
            plt.plot(self.time_record,self.total_distance_travelled_record)
            plt.xlabel('Time (s)')
            plt.ylabel('Total Distance Travelled (m)')

            plt.savefig('Distance_vs_Time_For_%s.png' % self.robot_namespace.replace('/',''))

            plt.clf()

            plt.plot(self.time_record, self.percent_of_map_covered_record)
            plt.xlabel('Time (s)')
            plt.ylabel('Percent of Map Explored')

            plt.savefig('Percent_Explored_vs_Time_For_%s.png' % self.robot_namespace.replace('/',''))

            plt.clf()

            plt.plot(self.total_distance_travelled_record, self.percent_of_map_covered_record)
            plt.xlabel('Total Distance Travelled (m)')
            plt.ylabel('Percent of Map Explored')

            plt.savefig('Percent_Explored_vs_Distance_For_%s.png' % self.robot_namespace.replace('/',''))

            self.files_written = True

            rospy.signal_shutdown("Finished Evaluation")


if __name__ == '__main__':
    node = exploration_eval()

    while not rospy.is_shutdown():
        pass