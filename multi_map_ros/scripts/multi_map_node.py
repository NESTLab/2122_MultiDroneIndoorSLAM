#!/usr/bin/env python3

import rospy
from multi_map_ros.srv import merge_srv, merge_srvResponse, merge_srvRequest
from multi_map_ros.msg import dag_node

class multiMapMerge():
    def __init__(self):
        # basic setup parameters
        robot_name = rospy.get_param('-robot_name', 'bebop')
        rate = float(rospy.get_param('~rate', '1.0'))
        string = str(robot_name)+'_merger'
        rospy.init_node(string, anonymous=True)

        # init service requiest
        self.map_merge_srv = rospy.Service('/map_merge', merge_srv, self.map_merge_callback)

        # init merge pub and subs
        self.map_merge_sub = rospy.Subscriber('/dag_listener', dag_node, self.dag_listener_callback)
        self.map_merge_pub = rospy.Publisher('/dag_publisher', dag_node, queue_size=1)
        print("multi map node created!")

    def map_merge_callback(self, req):
        # process map merge
        # determine if a merge can happen, if so begin to process sub
        return merge_srv_response(True)

    def dag_listener_callback(self, dag_node):
        # do dag listener things

        # publish a node based on response
        self.map_merge_pub.publish(dag_node())

if __name__ == "__main__":
    mmm = multiMapMerge()
    rospy.spin()

