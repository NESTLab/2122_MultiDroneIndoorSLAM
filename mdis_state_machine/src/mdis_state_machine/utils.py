import rospy
from subprocess import Popen, DEVNULL
from mdis_state_machine.msg import RobotsState
from mdis_state_machine.msg import Connection
from mdis_state_machine.msg import ConnectionRequest
from mdis_state_machine.msg import Location
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Empty
from typing import List
from roslaunch.parent import ROSLaunchParent

# State enums from robot_state_machine.h
IDLE = 0
GO_TO_EXPLORE = 1
EXPLORE = 2
GO_TO_MEET = 3
TRANSIT_TO_MEET = 4
MERGE_MAP = 5
DECIDE_NEXT_MEETING = 6
RECEIVE_NEXT_MEETING = 7
END_MEETING = 8
GO_TO_DUMP_DATA = 9
DUMP_DATA = 10
ERROR_STATE = 11



max_attempts_for_robot_message = 3
ideal_state_change_duration = 5
headstart_to_check = 3
max_time_to_wait_to_change_state = 1
message_wait_timeout = 1
robot_state_topic = "/robots_state"
connection_check_topic = "/connection_check"
testing_switch_trigger_topic = "/testing_switch_trigger"
testing_frontier_publish_topic = "/frontier_list"
testing_frontier_request_topic = "/frontier_request"
connection_request_topic = "/connection_request"

def gen_topic_name(lst: List[str]) -> str:
	result = ""
	for n in lst:
		result += "/" + n.replace("/", "")
	return result


def listenStateChange(init_state, robot_name):
	topic_name = gen_topic_name([robot_name, robot_state_topic])
	msg = rospy.wait_for_message(topic_name, RobotsState, timeout=message_wait_timeout)
	return msg.robot_state == init_state


def verifyConnStateChange(init_state, change_state, robot_name, robot_partner):
	init_state_sucs = listenStateChange(init_state, robot_name)

	parent_name = String(data=robot_partner)
	conn_robot_name = String(data=robot_name)
	robot_conn_msg = Connection()
	robot_conn_msg.connection_to = parent_name

	robot_conn_request_msg = ConnectionRequest()
	robot_conn_request_msg.robot_name = parent_name
	robot_conn_request_msg.connection_to = conn_robot_name

	connection_topic_name = gen_topic_name([robot_name, connection_check_topic])
	# interest_topic_name = gen_topic_name([robot_partner, connection_request_topic])
	pub = rospy.Publisher(connection_topic_name, Connection, queue_size=10)
	conn_req_pub = rospy.Publisher(connection_request_topic, ConnectionRequest, queue_size=10)
	rospy.sleep(0.5)
	pub.publish(robot_conn_msg)
	rospy.sleep(0.2)
	conn_req_pub.publish(robot_conn_request_msg)
	rospy.sleep(0.2)
	final_state_sucs = listenStateChange(change_state, robot_name)
	return final_state_sucs and init_state_sucs

def verifyTransitStateChange(init_state, change_state, robot_name, robot_partner):
	init_state_sucs = listenStateChange(init_state, robot_name)

	parent_name = String(data=robot_partner)
	conn_robot_name = String(data=robot_name)

	robot_conn_request_msg = ConnectionRequest()
	robot_conn_request_msg.robot_name = parent_name
	robot_conn_request_msg.connection_to = conn_robot_name
	robot_conn_request_msg.robot_state = init_state
	conn_req_pub = rospy.Publisher(connection_request_topic, ConnectionRequest, queue_size=10)
	rospy.sleep(0.5)
	conn_req_pub.publish(robot_conn_request_msg)
	rospy.sleep(1)
	
	final_state_sucs = listenStateChange(change_state, robot_name)
	return final_state_sucs and init_state_sucs



def verifyStateChange(init_state, change_state, robot_name):
	init_state_sucs = listenStateChange(init_state, robot_name)

	trig_switch_msg = Empty()
	
	trigger_switch_topic_name = gen_topic_name([robot_name, testing_switch_trigger_topic])
	pub = rospy.Publisher(trigger_switch_topic_name, Empty, queue_size=10)
	rospy.sleep(0.5)
	pub.publish(trig_switch_msg)
	rospy.sleep(0.5)

	final_state_sucs = listenStateChange(change_state, robot_name)
	return final_state_sucs and init_state_sucs

def verifyMeetingLocCalcStateChange(init_state, change_state, robot_name):
	init_state_sucs = listenStateChange(init_state, robot_name)
	
	topic_name = gen_topic_name([robot_name, testing_frontier_request_topic])
	msg = rospy.wait_for_message(topic_name, String, timeout=message_wait_timeout)
	if(msg.data == robot_name):
		frontier_publish_topic = gen_topic_name([robot_name, testing_frontier_publish_topic])
		pub = rospy.Publisher(frontier_publish_topic, PoseArray, queue_size=10)
		rospy.sleep(0.5)

		data = PoseArray()
		zero_pose = Pose()
		data.poses.append(zero_pose)
		pub.publish(data)
		rospy.sleep(1)

		final_state_sucs = listenStateChange(change_state, robot_name)
		return final_state_sucs and init_state_sucs

	return False


def start_state_machine(launch_file: str) -> ROSLaunchParent:
	parent = ROSLaunchParent("", [launch_file], is_core=True)     # run_id can be any string
	parent.start()
	return parent


def stop_state_machine(parent: ROSLaunchParent) -> None:
	parent.shutdown()
	while True:
		proc = Popen(["rostopic", "list"], stdout=DEVNULL, stderr=DEVNULL)
		if proc.wait() != 0:
			break
