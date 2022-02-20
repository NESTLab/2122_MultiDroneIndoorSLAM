#!/usr/bin/env python
import unittest
import rospy
import pathlib
import os

from mdis_state_machine import *

TEST_DIR = pathlib.Path(__file__).parent.absolute()
LAUNCH_DIR = os.path.join(TEST_DIR.parent.absolute(), 'launch')
RELAY_LAUNCH_FILE = os.path.join(LAUNCH_DIR, 'test_state_machine_relay.launch')
ROBOT_NAME = "test_robot_2"


class TestStateShiftRelay(unittest.TestCase):

	def test_init_state(self: unittest.TestCase) -> None:
		proc = start_state_machine(RELAY_LAUNCH_FILE)

		rospy.init_node("test_node")

		self.assertEqual(verifyInitState(GO_TO_MEET, ROBOT_NAME), True, "Initial state check for relay failed")

	# def test_triggered_states(self: unittest.TestCase) -> None:
		self.assertEqual(verifyConnStateChange(GO_TO_MEET, MEET, ROBOT_NAME), True, "State shift check for relay state 3-4 failed")

	# def test_timed_states(self: unittest.TestCase) -> None:
		self.assertEqual(verifyTimedStateChange(MEET, GO_TO_DUMP_DATA, ROBOT_NAME), True, "State shift check for relay state 4-5 failed")
		self.assertEqual(verifyTimedStateChange(GO_TO_DUMP_DATA, DUMP_DATA, ROBOT_NAME), True, "Second state shift check for relay state 5-6 failed")
		self.assertEqual(verifyTimedStateChange(DUMP_DATA, GO_TO_MEET, ROBOT_NAME), True, "Second state shift check for relay state 6-3 failed")

	# def test_second_loop(self: unittest.TestCase) -> None:
		self.assertEqual(verifyConnStateChange(GO_TO_MEET, MEET, ROBOT_NAME), True, "Second state shift check for relay state 3-4 failed")
		self.assertEqual(verifyTimedStateChange(MEET, GO_TO_DUMP_DATA, ROBOT_NAME), True, "State shift check for relay state 4-5 failed")
		self.assertEqual(verifyTimedStateChange(GO_TO_DUMP_DATA, DUMP_DATA, ROBOT_NAME), True, "Second state shift check for relay state 5-6 failed")
		self.assertEqual(verifyTimedStateChange(DUMP_DATA, GO_TO_MEET, ROBOT_NAME), True, "Second state shift check for relay state 6-3 failed")

		stop_state_machine(proc)


if __name__ == '__main__':
	unittest.main()
