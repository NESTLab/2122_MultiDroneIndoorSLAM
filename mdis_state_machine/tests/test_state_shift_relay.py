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
ROBOT_PARTNER = "test_robot_1"
DATA_CENTER = "data_center"


class TestStateShiftRelay(unittest.TestCase):

	def test_success_state(self: unittest.TestCase) -> None:
		proc = start_state_machine(RELAY_LAUNCH_FILE)
		rospy.init_node("test_node")

		self.assertEqual(listenStateChange(GO_TO_MEET, ROBOT_NAME), True, "Initial state check for relay failed")

		self.assertEqual(verifyConnStateChange(GO_TO_MEET, TRANSIT_TO_MEET, ROBOT_NAME, ROBOT_PARTNER), True, "State shift check for relay state 3-4 failed")
		self.assertEqual(verifyTransitStateChange(TRANSIT_TO_MEET, MERGE_MAP, ROBOT_NAME, ROBOT_PARTNER), True, "State shift check for relay state 4-5 failed")
		self.assertEqual(verifyStateChange(MERGE_MAP, RECEIVE_NEXT_MEETING, ROBOT_NAME), True, "State shift check for relay state 5-7 failed")
		self.assertEqual(verifyTransitStateChange(RECEIVE_NEXT_MEETING, END_MEETING, ROBOT_NAME, ROBOT_PARTNER), True, "State shift check for relay state 7-8 failed")
		self.assertEqual(verifyTransitStateChange(END_MEETING, GO_TO_DUMP_DATA, ROBOT_NAME, ROBOT_PARTNER), True, "State shift check for relay state 8-9 failed")
		self.assertEqual(verifyConnStateChange(GO_TO_DUMP_DATA, TRANSIT_TO_MEET, ROBOT_NAME, DATA_CENTER), True, "State shift check for relay state 9-4 failed")
		self.assertEqual(verifyTransitStateChange(TRANSIT_TO_MEET, MERGE_MAP, ROBOT_NAME, DATA_CENTER), True, "State shift check for relay state 4-5 (2) failed")
		self.assertEqual(verifyStateChange(MERGE_MAP, END_MEETING, ROBOT_NAME), True, "State shift check for relay state 5-8 failed")
		self.assertEqual(verifyTransitStateChange(END_MEETING, GO_TO_MEET, ROBOT_NAME, DATA_CENTER), True, "State shift check for relay state 8-3 failed")
		self.assertEqual(verifyConnStateChange(GO_TO_MEET, TRANSIT_TO_MEET, ROBOT_NAME, ROBOT_PARTNER), True, "State shift check for relay state 3-4 failed")
		stop_state_machine(proc)

if __name__ == '__main__':
	unittest.main()
