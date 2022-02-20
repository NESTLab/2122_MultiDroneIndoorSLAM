import rospy
from mdis_state_machine.msg import RobotsState
from mdis_state_machine.msg import Connection
from std_msgs.msg import String
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
max_time_to_wait_to_change_state = 6
message_wait_timeout = 30
robot_state_topic = "/robots_state"


def verifyInitState(init_state, robot_name):
	for i in range(max_attempts_for_robot_message):
		msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
		if msg.robot_name.data == robot_name:
			return msg.robot_state == init_state
	return False


def verifyTimedStateChange(init_state, change_state, robot_name):
	rospy.sleep(ideal_state_change_duration - headstart_to_check)
	time_start = rospy.get_rostime().secs
	init_state_sucs = False
	while rospy.get_rostime().secs < time_start + max_time_to_wait_to_change_state:
		msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
		if msg.robot_name.data == robot_name:
			if msg.robot_state == init_state:
				init_state_sucs = True
			if msg.robot_state == change_state:
				return True and init_state_sucs
	return False


def verifyConnStateChange(init_state, change_state, robot_name):
	rospy.sleep(ideal_state_change_duration - headstart_to_check)
	init_state_sucs = False
	# while rospy.get_rostime().secs < time_start+max_time_to_wait_to_change_state:
	msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
	if msg.robot_name.data == robot_name:
		if msg.robot_state == init_state:
			init_state_sucs = True

	robot_conn_msg = Connection()
	robot_conn_msg.connection_between = []
	robot_conn_msg.connection_between.append(String(data=robot_name))
	robot_conn_msg.connection_between.append(String(data="dummy_parent"))

	pub = rospy.Publisher('/connection_check', Connection, queue_size=10)
	time_start = rospy.get_rostime().secs
	while rospy.get_rostime().secs < time_start + max_time_to_wait_to_change_state:
		pub.publish(robot_conn_msg)
		rospy.sleep(0.5)
		msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
		if msg.robot_name.data == robot_name:
			if msg.robot_state == change_state:
				return True and init_state_sucs
	return False


def start_state_machine(launch_file: str) -> ROSLaunchParent:
	parent = ROSLaunchParent("", [launch_file], is_core=True)     # run_id can be any string
	parent.start()
	return parent


def stop_state_machine(parent: ROSLaunchParent) -> None:
	parent.shutdown()
