import rospy
from subprocess import Popen, DEVNULL
from mdis_state_machine.msg import RobotsState
from mdis_state_machine.msg import Connection
from mdis_state_machine.msg import ConnectionRequest
from mdis_state_machine.msg import Location
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Empty
from typing import List
from roslaunch.parent import ROSLaunchParent

# State enums from robot_state_machine.h
IDLE = 0
GO_TO_EXPLORE = 1
EXPLORE = 2
GO_TO_MEET = 3
MEET = 4
GO_TO_DUMP_DATA = 5
DUMP_DATA = 6

max_attempts_for_robot_message = 10
ideal_state_change_duration = 5
headstart_to_check = 3
max_time_to_wait_to_change_state = 1
message_wait_timeout = 1
robot_state_topic = "/robots_state"
connection_check_topic = "/connection_check"
testing_switch_trigger_topic = "/testing_switch_trigger"
connection_request_topic = "/connection_request"

def gen_topic_name(lst: List[str]) -> str:
	result = ""
	for n in lst:
		result += "/" + n.replace("/", "")
	return result


def verifyInitState(init_state, robot_name):
	topic_name = gen_topic_name([robot_name, robot_state_topic])
	for i in range(max_attempts_for_robot_message):
		msg = rospy.wait_for_message(topic_name, RobotsState, timeout=message_wait_timeout)
		return msg.robot_state == init_state
	return False


def verifyTimedStateChange(init_state, change_state, robot_name):
	rospy.sleep(ideal_state_change_duration - headstart_to_check)
	time_start = rospy.get_rostime().secs
	init_state_sucs = False
	topic_name = gen_topic_name([robot_name, robot_state_topic])
	while rospy.get_rostime().secs < time_start + max_time_to_wait_to_change_state:
		msg = rospy.wait_for_message(topic_name, RobotsState, timeout=message_wait_timeout)
		if msg.robot_name.data == robot_name:
			if msg.robot_state == init_state:
				init_state_sucs = True
			if msg.robot_state == change_state:
				return True and init_state_sucs
	return False


def verifyConnStateChange(init_state, change_state, robot_name, robot_partner):
	init_state_sucs = False
	state_topic_name = gen_topic_name([robot_name, robot_state_topic])
	msg = rospy.wait_for_message(state_topic_name, RobotsState, timeout=message_wait_timeout)
	if msg.robot_state == init_state:
		init_state_sucs = True

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
	rospy.sleep(0.2)
	pub.publish(robot_conn_msg)
	rospy.sleep(0.2)
	conn_req_pub.publish(robot_conn_request_msg)
	rospy.sleep(0.2)
	msg = rospy.wait_for_message(state_topic_name, RobotsState, timeout=message_wait_timeout)	
	if msg.robot_state == change_state:
		return True and init_state_sucs
	return False

def verifyStateChange(init_state, change_state, robot_name):
	init_state_sucs = False
	# while rospy.get_rostime().secs < time_start+max_time_to_wait_to_change_state:
	state_topic_name = gen_topic_name([robot_name, robot_state_topic])  
	msg = rospy.wait_for_message(state_topic_name, RobotsState, timeout=message_wait_timeout)	
	if msg.robot_state == init_state:
		init_state_sucs = True

	trig_switch_msg = Empty()
	
	trigger_switch_topic_name = gen_topic_name([robot_name, testing_switch_trigger_topic])
	pub = rospy.Publisher(trigger_switch_topic_name, Empty, queue_size=10)
	rospy.sleep(0.5)
	pub.publish(trig_switch_msg)
	rospy.sleep(0.5)
	msg = rospy.wait_for_message(state_topic_name, RobotsState, timeout=message_wait_timeout)
	if msg.robot_state == change_state:
		return True and init_state_sucs

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
