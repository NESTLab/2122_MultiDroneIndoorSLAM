#!/usr/bin/env python
import unittest
import rospy
from mdis_state_machine.msg import RobotsState
from mdis_state_machine.msg import Connection
from std_msgs.msg import String

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
robot_name = "test_robot_1"


def verifyInitState(init_state):
  for i in range(max_attempts_for_robot_message):
    msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
    if msg.robot_name.data == robot_name:
      return msg.robot_state == init_state
  return False


def verifyTimedStateChange(init_state, change_state):
  rospy.sleep(ideal_state_change_duration-headstart_to_check)
  time_start = rospy.get_rostime().secs
  init_state_sucs = False
  while rospy.get_rostime().secs < time_start+max_time_to_wait_to_change_state:
    msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
    if msg.robot_name.data == robot_name:
      if msg.robot_state == init_state:
        init_state_sucs = True
      if msg.robot_state == change_state:
        return True and init_state_sucs 
  return False

def verifyConnStateChange(init_state, change_state):
  rospy.sleep(ideal_state_change_duration-headstart_to_check)
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
  while rospy.get_rostime().secs < time_start+max_time_to_wait_to_change_state:
    pub.publish(robot_conn_msg)
    rospy.sleep(0.5)
    msg = rospy.wait_for_message(robot_state_topic, RobotsState, timeout=message_wait_timeout)
    if msg.robot_name.data == robot_name:
      if msg.robot_state == change_state:
        return True and init_state_sucs 
  return False
  


class TestPing(unittest.TestCase):

  def test_init_state(self: unittest.TestCase) -> None:
    self.assertEqual(verifyInitState(1), True, "Initial state check for explorer failed")
  
  # def test_timed_states(self: unittest.TestCase) -> None:
    self.assertEqual(verifyTimedStateChange(1, 2), True, "State shift check for explorer state 1-2 failed")
    self.assertEqual(verifyTimedStateChange(2, 3), True, "State shift check for explorer state 2-3 failed")
  
  # def test_triggered_states(self: unittest.TestCase) -> None:
    self.assertEqual(verifyConnStateChange(3, 4), True, "State shift check for explorer state 3-4 failed")
  
  # def test_second_loop(self: unittest.TestCase) -> None:
    self.assertEqual(verifyTimedStateChange(4, 1), True, "State shift check for explorer state 4-1 failed")
    self.assertEqual(verifyTimedStateChange(1, 2), True, "Second state shift check for explorer state 1-2 failed")
    self.assertEqual(verifyTimedStateChange(2, 3), True, "Second state shift check for explorer state 2-3 failed")
    self.assertEqual(verifyConnStateChange(3, 4), True, "Second state shift check for explorer state 3-4 failed")

if __name__ == '__main__':
    rospy.init_node("test_node")
    unittest.main()