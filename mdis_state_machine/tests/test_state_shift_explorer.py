#!/usr/bin/env python
import unittest
import rospy
import pathlib
import os
from mdis_state_machine import *


TEST_DIR = pathlib.Path(__file__).parent.absolute()
LAUNCH_DIR = os.path.join(TEST_DIR.parent.absolute(), 'launch')
EXPLORER_LAUNCH_FILE = os.path.join(LAUNCH_DIR, 'test_state_machine_explorer.launch')
ROBOT_NAME = "test_robot_1"
ROBOT_PARTNER = "test_robot_2"


class TestStateShiftExplorer(unittest.TestCase):

	def test_init_state(self: unittest.TestCase) -> None:
		proc = start_state_machine(EXPLORER_LAUNCH_FILE)
		rospy.init_node("test_node")

		self.assertEqual(verifyInitState(GO_TO_EXPLORE, ROBOT_NAME), True, "Initial state check for explorer failed")

		# def test_timed_states(self: unittest.TestCase) -> None:
		self.assertEqual(verifyStateChange(GO_TO_EXPLORE, EXPLORE, ROBOT_NAME), True, "State shift check for explorer state 1-2 failed")
		self.assertEqual(verifyStateChange(EXPLORE, GO_TO_MEET, ROBOT_NAME), True, "State shift check for explorer state 2-3 failed")
		# self.assertEqual(verifyTimedStateChange(GO_TO_EXPLORE, EXPLORE, ROBOT_NAME), True, "State shift check for explorer state 1-2 failed")
		# self.assertEqual(verifyTimedStateChange(EXPLORE, GO_TO_MEET, ROBOT_NAME), True, "State shift check for explorer state 2-3 failed")

		# # def test_triggered_states(self: unittest.TestCase) -> None:
		# self.assertEqual(verifyConnStateChange(GO_TO_MEET, MEET, ROBOT_NAME, ROBOT_PARTNER), True, "State shift check for explorer state 3-4 failed")

		# # def test_second_loop(self: unittest.TestCase) -> None:
		# self.assertEqual(verifyTimedStateChange(MEET, GO_TO_EXPLORE, ROBOT_NAME), True, "State shift check for explorer state 4-1 failed")
		# self.assertEqual(verifyTimedStateChange(GO_TO_EXPLORE, EXPLORE, ROBOT_NAME), True, "Second state shift check for explorer state 1-2 failed")
		# self.assertEqual(verifyTimedStateChange(EXPLORE, GO_TO_MEET, ROBOT_NAME), True, "Second state shift check for explorer state 2-3 failed")
		# self.assertEqual(verifyConnStateChange(GO_TO_MEET, MEET, ROBOT_NAME), True, "Second state shift check for explorer state 3-4 failed")
		stop_state_machine(proc)


if __name__ == '__main__':
	unittest.main()
