#
#
#
#
#
#
#
# def send_to_location(self, pt):
#     move_msg = PoseStamped()
#     move_msg.header.frame_id = self.robot_namespace_no_slash + '/map'
#
#     move_msg.pose.position.x = pt[0]
#     move_msg.pose.position.y = pt[1]
#     move_msg.pose.position.z = 0
#
#     move_msg.pose.orientation.x = 0
#     move_msg.pose.orientation.y = 0
#     move_msg.pose.orientation.z = 0
#     move_msg.pose.orientation.w = 1
#
#     self.visualize_goal_point(move_msg)
#
#     if self.check_pose_possible(move_msg):
#         print("sending to pose")
#         self.move_pub.publish(move_msg)
#         self.wait_until_move_achieved(move_msg)
#     else:
#         rospy.logwarn("Goal pose is impossible to plan to: " + str(move_msg))
#
#
# def show_location(self, pt):
#     move_msg = PoseStamped()
#     move_msg.header.frame_id = self.robot_namespace_no_slash + '/map'
#
#     move_msg.pose.position.x = pt[0]
#     move_msg.pose.position.y = pt[1]
#     move_msg.pose.position.z = 0
#
#     move_msg.pose.orientation.x = 0
#     move_msg.pose.orientation.y = 0
#     move_msg.pose.orientation.z = 0
#     move_msg.pose.orientation.w = 1
#
#     self.visualize_goal_point(move_msg)
#
#
# def wait_until_move_achieved(self, goal_pose):
#     rospy.loginfo("waiting for move base to finish goal")
#
#     # while not rospy.is_shutdown():
#     #     curr_odom = self.get_odom()
#     #     curr_x = curr_odom.pose.pose.position.x
#     #     curr_y = curr_odom.pose.pose.position.y
#     #
#     #     goal_x = goal_pose.pose.position.x
#     #     goal_y = goal_pose.pose.position.y
#     #
#     #     curr_map = self.get_map()
#     #
#     #     index_of_robot = int(self.point_to_index((curr_x, curr_y), curr_map))
#     #     pt = self.index_to_point(index_of_robot, curr_map)
#     #     curr_location_map_frame = self.map_to_world(pt[0], pt[1], curr_map)
#     #
#     #     print("CURRENT LOCATION: " + str(curr_location_map_frame))
#     #     print("GOAL LOCATION: (" + str(goal_x) + ", " + str(goal_y) + ")")
#     #
#     #     if abs(goal_x-curr_location_map_frame[0]) <= self.move_base_threshold and abs(goal_y-curr_location_map_frame[1]) <= self.move_base_threshold:
#     #         rospy.loginfo("Robot has reached desired pose")
#     #         print("Robot has reached desired pose")
#     #         break
#     #
#     #     # elif self.is_stuck():
#     #     #     rospy.logerr("Robot is stuck")
#     #     #     print("Robot is stuck")
#     #     #     break
#
#     self.record_move_base_response = True
#     while not rospy.is_shutdown():
#
#         if self.move_base_result is not None:
#             text = self.move_base_result.status.text
#             self.record_move_base_response = False
#             self.move_base_result = None
#             if text == "Goal reached.":
#                 rospy.logwarn("GOAL REACHED!!!")
#                 print("GOAL REACHED!!!")
#                 break
#             else:
#                 rospy.logwarn("GOAL _NOT_ REACHED!!!")
#                 print("GOAL _NOT_ REACHED!!!")
#                 self.goal_delete_pub.publish(GoalID())
#                 break
#
#         # elif self.is_stuck():
#         #     rospy.logerr("Robot is stuck")
#         #     print("Robot is stuck")
#         #     self.goal_delete_pub.publish(GoalID())
#         #     break
#
#
# def check_pose_possible(self, goal_pose):
#     move_base_service_topic = self.robot_namespace + '/move_base/make_plan'
#     print("waiting for %s service", move_base_service_topic)
#
#     rospy.wait_for_service(move_base_service_topic)
#
#     req = GetPlan()
#
#     curr_odom = self.get_odom()
#     start_pose = PoseStamped()
#     start_pose.header.frame_id = goal_pose.header.frame_id
#
#     start_pose.pose.position.x = curr_odom.pose.pose.position.x
#     start_pose.pose.position.y = curr_odom.pose.pose.position.y
#     start_pose.pose.position.z = 0
#
#     start_pose.pose.orientation.x = 0
#     start_pose.pose.orientation.y = 0
#     start_pose.pose.orientation.z = 0
#     start_pose.pose.orientation.w = 1
#
#     req.start = start_pose
#     req.goal = goal_pose
#     req.tolerance = .5
#
#     try:
#         get_plan = rospy.ServiceProxy(move_base_service_topic, GetPlan)
#         resp = get_plan(req.start, req.goal, req.tolerance)
#         rospy.loginfo(resp)
#
#         if len(resp.plan.poses) > 0:
#             rospy.loginfo("PATH FOUND - goal is possible to get to")
#             return True
#         else:
#             rospy.loginfo("NO path found - goal is NOT possible to get to")
#             return False
#
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)
#         rospy.logerr("this means a goal is currently executing - no plan can be made right now")
#         return False
#
#
# def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate):
#     """
#     We wait for the cmd_vel twist given to be reached by the robot reading
#     from the odometry.
#     :param cmd_vel_value: Twist we want to wait to reach.
#     :param epsilon: Error acceptable in odometry readings.
#     :param update_rate: Rate at which we check the odometry.
#     :return:
#     """
#     rospy.logdebug("START wait_until_twist_achieved...")
#
#     rate = rospy.Rate(update_rate)
#     start_wait_time = rospy.get_rostime().to_sec()
#     end_wait_time = 0.0
#     epsilon = 0.05
#
#     rospy.logdebug("Desired Twist Cmd>>" + str(cmd_vel_value))
#     rospy.logdebug("epsilon>>" + str(epsilon))
#
#     linear_speed = cmd_vel_value.linear.x
#     angular_speed = cmd_vel_value.angular.z
#
#     linear_speed_plus = linear_speed + epsilon
#     linear_speed_minus = linear_speed - epsilon
#     angular_speed_plus = angular_speed + epsilon
#     angular_speed_minus = angular_speed - epsilon
#
#     while not rospy.is_shutdown():
#         current_odometry = self._check_odom_ready()
#         # IN turtlebot3 the odometry angular readings are inverted, so we have to invert the sign.
#         odom_linear_vel = current_odometry.twist.twist.linear.x
#         odom_angular_vel = -1 * current_odometry.twist.twist.angular.z
#
#         rospy.logdebug("Linear VEL=" + str(odom_linear_vel) + ", ?RANGE=[" + str(linear_speed_minus) + "," + str(
#             linear_speed_plus) + "]")
#         rospy.logdebug("Angular VEL=" + str(odom_angular_vel) + ", ?RANGE=[" + str(angular_speed_minus) + "," + str(
#             angular_speed_plus) + "]")
#
#         linear_vel_are_close = (odom_linear_vel <= linear_speed_plus) and (odom_linear_vel > linear_speed_minus)
#         angular_vel_are_close = (odom_angular_vel <= angular_speed_plus) and (odom_angular_vel > angular_speed_minus)
#
#         if linear_vel_are_close and angular_vel_are_close:
#             rospy.logdebug("Reached Velocity!")
#             end_wait_time = rospy.get_rostime().to_sec()
#             break
#         rospy.logdebug("Not there yet, keep waiting...")
#         rate.sleep()
#     delta_time = end_wait_time - start_wait_time
#     rospy.logdebug("[Wait Time=" + str(delta_time) + "]")
#
#     rospy.logdebug("END wait_until_twist_achieved...")
#
#     return delta_time
#
#
# def is_stuck(self):
#     x_list = list()
#     y_list = list()
#     z_list = list()
#     recorded_odom = self.odom_recorder
#     for odom in recorded_odom:
#         x_list.append(odom.pose.pose.position.x)
#         y_list.append(odom.pose.pose.position.y)
#         z_list.append(odom.pose.pose.position.z)
#
#     if abs(max(x_list) - min(x_list)) < self.odom_diff_threshold and abs(
#             max(y_list) - min(y_list)) < self.odom_diff_threshold and abs(
#             max(z_list) - min(z_list)) < self.odom_diff_threshold:
#         return True
#     else:
#         return False
#
#
# def get_odom(self):
#     return self.odom
#
#
# def get_imu(self):
#     return self.imu
#
#
# def get_laser_scan(self):
#     return self.laser_scan
#
#
# def get_frontier_map(self):
#     # while not self.frontier_map_receivedt:
#     #     pass
#     return self.frontier_map
#
#
# def get_map(self):
#     # while not self.map_received:
#     #     pass
#     return self.map
#
#
# def get_pose_map(self):
#     # while not self.pose_map_received:
#     #     pass
#     return self.pose_map
#
#
# def get_frontier_map_set(self):
#     return self.frontier_map_received
#
#
# def get_map_set(self):
#     return self.map_received
#
#
# def get_pose_map_set(self):
#     return self.pose_map_received
#
#
# def pub_reset_gmapping(self):
#     reset_msg = String()
#     reset_msg.data = 'reset'
#     self.reset_gmapping_pub.publish(reset_msg)
#     print("Sent reset signal")
#
#
# def visualize_goal_point(self, marker_pose):
#     marker = Marker()
#     marker.action = Marker.ADD
#     marker.header.frame_id = self.robot_namespace_no_slash + "/map"  # self.robot_namespace_no_slash + "/base_link"
#     marker.id = 0
#     marker.type = Marker.SPHERE
#     marker.pose.position.x = marker_pose.pose.position.x
#     marker.pose.position.y = marker_pose.pose.position.y
#     marker.pose.position.z = 0
#     marker.pose.orientation.x = 0
#     marker.pose.orientation.y = 0
#     marker.pose.orientation.z = 0
#     marker.pose.orientation.w = 1
#     marker.scale.x = 1.0
#     marker.scale.y = 1.0
#     marker.scale.z = 1.0
#
#     marker.color.a = 0.5
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#
#     self.visualization_pub.publish(marker)
#
#
# def point_to_index(self, point, my_map):
#     """convert a index to a point"""
#     pt = self.world_to_map(point[0], point[1], my_map)
#     return pt[1] * my_map.info.width + pt[0]
#
#
# def convert_location(self, loc, my_map):
#     """converts points to the grid"""
#     res = my_map.info.resolution
#     Xorigin = my_map.info.origin.position.x
#     Yorigin = my_map.info.origin.position.y
#
#     # offset from origin and then divide by resolution
#     Xcell = int((loc[0] - Xorigin - (res / 2)) / res)
#     Ycell = int((loc[1] - Yorigin - (res / 2)) / res)
#     return (Xcell, Ycell)
#
#
# def index_to_point(self, index, my_map):
#     x = index % int(my_map.info.width)
#     y = (index - x) / my_map.info.width
#     return (x, y)
#
#
# def map_to_world(self, x, y, my_map):
#     """
#         converts a point from the map to the world
#         :param x: float of x position
#         :param y: float of y position
#         :return: tuple of converted point
#     """
#     resolution = my_map.info.resolution
#
#     originX = my_map.info.origin.position.x
#     originY = my_map.info.origin.position.y
#
#     # multiply by resolution, then move by origin offset
#     x = x * resolution + originX + resolution / 2
#     y = y * resolution + originY + resolution / 2
#     return (x, y)
#
#
# def world_to_map(self, x, y, my_map):
#     """
#         converts a point from the world to the map
#         :param x: float of x position
#         :param y: float of y position
#         :return: tuple of converted point
#     """
#
#     return self.convert_location((x, y), my_map)
#
#
# def start_move_base(self):
#     self.start_move_base_pub.publish("start")
#
#
# def stop_move_base(self):
#     self.stop_move_base_pub.publish("stop")