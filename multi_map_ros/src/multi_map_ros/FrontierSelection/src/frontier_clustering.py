#!/usr/bin/env python3.8

import rospy

import numpy as np

import copy

from scipy import signal as sig

from std_msgs.msg import Header, String

from nav_msgs.msg import OccupancyGrid, GridCells

from geometry_msgs.msg import Point, PoseStamped, Pose, PoseArray

from tf import TransformListener

from rospy.exceptions import ROSTimeMovedBackwardsException, ROSInterruptException

SEND_TO_FRONTIERS = False

SEND_STATE_MACHINE_FRONTIERS = True

SM_LIMIT = 10

class find_frontier:

    def __init__(self):

        rospy.init_node('find_fontier')

        self.cell_threshold = 0

        self.threshold_multiplier = 2

        # self.tf_listener_ = TransformListener()

        self.weight_output = .75

        self.clusters = list()

        self.max_msg_count_before_start = 10

        self.curr_msg_count = 0

        self.robot_namespace = '/tb3_0'

        self.publish_next_frontier = False

        self.publish_ns = ""

        # self.timer = rospy.Timer(rospy.Duration(5), self.frontier_callback)

        self.init_subscribers_and_publishers(self.robot_namespace)

        rospy.loginfo("FINISHED INIT")

    def init_subscribers_and_publishers(self, ns):

        self.mapSub = rospy.Subscriber(ns + '/map', OccupancyGrid, self.map_callback)

        self.fringe = rospy.Publisher('/map_fringe', GridCells, queue_size=10)

        self.final_frontier_pub = rospy.Publisher('/selected_frontier', GridCells, queue_size=10)

        self.center_fringe_pub = rospy.Publisher('/selected_center', GridCells, queue_size=10)

        self.move_pub = rospy.Publisher(ns + '/move_base_simple/goal', PoseStamped, queue_size=10)

        self.move_helper_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.helper)

        self.move_helper_pub = rospy.Publisher(ns + '/move_base_simple/goal', PoseStamped,

                                               queue_size=10)

        self.frontier_center_pub = rospy.Publisher(ns + '/frontier_list', PoseArray, queue_size=10)

        self.frontier_center_request = rospy.Subscriber('/frontier_request', String, self.request_callback)

    def request_callback(self, msg):

        ns = str(msg.data).replace("\"","")

        if self.robot_namespace != ns:

            self.robot_namespace = ns

            self.init_subscribers_and_publishers(ns)

        self.publish_ns = ns

        self.publish_next_frontier = True

    def helper(self, msg):

        # self.move_helper_pub.publish(msg)
        pass

    def map_callback(self, msg):

        self.map = msg

        # if self.curr_msg_count > self.max_msg_count_before_start:

        #     self.frontier_callback(0)

        #     self.curr_msg_count = 0

        # self.curr_msg_count += 1

        # rospy.loginfo("curr message count is: " + str(self.curr_msg_count))

        if SEND_STATE_MACHINE_FRONTIERS:

            if self.publish_next_frontier and self.publish_ns in str(msg.header.frame_id):

                rospy.loginfo("Publishing frontiers for: " + str(self.publish_ns))

                self.frontier_callback(0)

            self.publish_next_frontier = False

        else:

            self.frontier_callback(0)

    def frontier_callback(self, timer_var):

        msg = self.map

        fringe_points = self.find_fringe(msg)

        if SEND_TO_FRONTIERS:

            all_frontier_clusters = self.cluster_cells(fringe_points)

            best_frontier_cells = self.calculate_least_cost_frontier(all_frontier_clusters)

            self.send_to_frontier(best_frontier_cells, msg)

        elif SEND_STATE_MACHINE_FRONTIERS:

            all_frontier_clusters = self.cluster_cells(fringe_points)

            centers = self.get_centers(all_frontier_clusters)

            self.send_centers_to_pub(centers)

    def find_fringe(self, msg):

        res = msg.info.resolution

        width = msg.info.width

        height = msg.info.height

        data = np.asarray(msg.data).reshape((width,height), order='F')

        expl = (data > -1)

        # Takes in a list of (x,y) locations to paint as explored nodes

        gridCells = GridCells()

        gridCells.cell_width = res

        gridCells.cell_height = res

        self.cell_threshold = self.threshold_multiplier*res

        header = Header()

        header.frame_id = msg.header.frame_id

        gridCells.header = header

        filter = np.asarray([-1,0,1,-2,0,2,-1,0,1]).reshape((3,3))

        edgeX = sig.convolve2d(expl, filter, mode='same', fillvalue=-1)

        edgeY = sig.convolve2d(expl, filter.T, mode='same', fillvalue=-1)

        edge = np.sqrt(np.add(np.power(edgeX, 2), np.power(edgeY, 2)))

        open_cells = np.logical_and((data > -1), (data < 10))

        fringe = np.logical_and((edge >= 2), open_cells)

        fringe_list = np.transpose(np.where(fringe))

        pts = []

        raw_pts = []

        for node in fringe_list:

            tup = self.map_to_world(node[0], node[1], msg)

            pt = Point(tup[0], tup[1], 0)

            raw_pt = (tup[0], tup[1], 0)

            pts.append(pt)

            raw_pts.append(raw_pt)

        gridCells.cells = pts

        self.fringe.publish(gridCells)

        return raw_pts

    def map_to_world(self, x, y, my_map):

        """

            converts a point from the map to the world

            :param x: float of x position

            :param y: float of y position

            :return: tuple of converted point

            @Author tberg1234 (from rbe3002)

        """

        resolution = my_map.info.resolution

        originX = my_map.info.origin.position.x

        originY = my_map.info.origin.position.y

        # multiply by resolution, then move by origin offset

        x = x * resolution + originX + resolution / 2

        y = y * resolution + originY + resolution / 2

        return (x, y)

    def cluster_cells(self, gridCells):

        all_frontier_clusters = dict()  # keeps track of clusters and the parent

        remaining_cells = copy.deepcopy(gridCells)  # cells remaining to be assigned to a group

        for a_cell in gridCells:  # in remaining_cells worked kinda but it was faster sooo

            if remaining_cells:

                assigned = False

                if all_frontier_clusters:

                    for key in all_frontier_clusters:

                        cells_in_cluster = all_frontier_clusters.get(key)

                        iterable_cells = copy.deepcopy(cells_in_cluster)

                        for other_cell in iterable_cells:

                            if np.linalg.norm(np.asarray(a_cell) - np.asarray(other_cell)) <= self.cell_threshold:

                                if a_cell not in cells_in_cluster:

                                    assigned = True

                                    cells_in_cluster.append(a_cell)

                                    if a_cell in remaining_cells:

                                        remaining_cells.remove(a_cell)

                                    # all_frontier_clusters[key] = cells_in_cluster

                                else:

                                    assigned = True

                                for potential_cell in remaining_cells:  # in gridCells

                                    if np.linalg.norm(np.asarray(a_cell) - np.asarray(potential_cell)) <= self.cell_threshold:

                                        if potential_cell not in cells_in_cluster:

                                            cells_in_cluster.append(potential_cell)

                                            if potential_cell in remaining_cells:

                                                remaining_cells.remove(potential_cell)

                                            # all_frontier_clusters[key] = cells_in_cluster

                        all_frontier_clusters[key] = cells_in_cluster

                    if not assigned:

                        first_list = list()

                        first_list.append(a_cell)

                        remaining_cells.remove(a_cell)

                        all_frontier_clusters[a_cell] = first_list

                else:

                    first_list = list()

                    first_list.append(a_cell)

                    remaining_cells.remove(a_cell)

                    all_frontier_clusters[a_cell] = first_list

            else:

                pass

        return all_frontier_clusters

    def calculate_least_cost_frontier(self, all_frontier_clusters):

        # max_key, max_value = max(all_frontier_clusters.items(), key=lambda x: len(set(x[1])))

        # return max_key, max_value

        cost_dict = dict()

        length_dict = {key: len(value) for key, value in all_frontier_clusters.items()}

        robot_position = self.get_current_position_in_map()

        for key in all_frontier_clusters.keys():

            cells = all_frontier_clusters.get(key)

            size = length_dict.get(key)

            center = np.average(cells, axis=0)

            distance = np.linalg.norm(np.asarray(center) - np.asarray(robot_position))

            cost = self.weight_output*distance + (1-self.weight_output)*(1-size)

            cost_dict[key] = cost

        sorted_costs = sorted(cost_dict, key=cost_dict.get, reverse=False)

        return all_frontier_clusters.get(sorted_costs[0])

    def get_centers(self, all_frontier_clusters):

        cost_dict = dict()

        length_dict = {key: len(value) for key, value in all_frontier_clusters.items()}

        robot_position = self.get_current_position_in_map()

        for key in all_frontier_clusters.keys():

            cells = all_frontier_clusters.get(key)

            size = length_dict.get(key)

            center = np.average(cells, axis=0)

            distance = np.linalg.norm(np.asarray(center) - np.asarray(robot_position))

            cost = self.weight_output * distance + (1 - self.weight_output) * (1 - size)

            cost_dict[key] = cost

        sorted_costs = sorted(cost_dict, key=cost_dict.get, reverse=False)

        centers = list()

        for cost in sorted_costs:

            fr = all_frontier_clusters.get(cost)

            c = np.average(fr, axis=0)

            centers.append(c)

        return centers

    def send_centers_to_pub(self, centers):

        centers_for_pub = []

        counter = 0

        for c in centers:

            if counter > SM_LIMIT:

                break

            move_msg = Pose()

            move_msg.position.x = c[0]

            move_msg.position.y = c[1]

            move_msg.position.z = c[2]

            move_msg.orientation.x = 0

            move_msg.orientation.y = 0

            move_msg.orientation.z = 0

            move_msg.orientation.w = 1

            centers_for_pub.append(move_msg)

            counter += 1

        full_msg = PoseArray()

        full_msg.poses = centers_for_pub

        full_msg.header.frame_id = self.robot_namespace + '/map'

        self.frontier_center_pub.publish(full_msg)

        rospy.loginfo("sent frontier list")

    def send_to_frontier(self, best_frontier_cells, msg):

        final_frontier = list()

        for cell in best_frontier_cells:

            final_frontier.append(Point(cell[0], cell[1], cell[2]))

        res = msg.info.resolution

        gridCells = GridCells()

        gridCells.cell_width = res

        gridCells.cell_height = res

        header = Header()

        header.frame_id = msg.header.frame_id

        gridCells.header = header

        gridCells.cells = final_frontier

        self.final_frontier_pub.publish(gridCells)

        center = np.average(best_frontier_cells, axis=0)

        center_lst = list()

        center_lst.append(Point(center[0], center[1], center[2]))

        gridCells.cells = center_lst

        self.center_fringe_pub.publish(gridCells)

        move_msg = PoseStamped()

        move_msg.header.frame_id = 'map'

        move_msg.pose.position.x = center[0]

        move_msg.pose.position.y = center[1]

        move_msg.pose.position.z = center[2]

        rospy.loginfo("SENDING TO FRONTIER: " + str(center))

        move_msg.pose.orientation.x = 0

        move_msg.pose.orientation.y = 0

        move_msg.pose.orientation.z = 0

        move_msg.pose.orientation.w = 1

        # self.move_pub.publish(move_msg)

    def get_current_position_in_map(self):

        # Position of the robot base in the map

        p1 = PoseStamped()

        p1.header.frame_id = self.robot_namespace+"/base_link"

        p1.pose.orientation.w = 1.0  # Neutral orientation

        # try:

        #     tf_listener_ = TransformListener()

        #     p_in_base = tf_listener_.transformPose(self.robot_namespace + "/map", p1)

        # except ROSTimeMovedBackwardsException:

        #     p_in_base = PoseStamped()

        #     print("ROS Interrupt Exception! Just ignore the exception!")

        p_in_base = PoseStamped()

        # p_in_base = self.tf_listener_.transformPose(self.robot_namespace+"/map", p1)

        return (p_in_base.pose.position.x, p_in_base.pose.position.y, p_in_base.pose.position.z)

if __name__ == '__main__':

    node = find_frontier()

    while not rospy.is_shutdown():

        pass

