#!/usr/bin/env python
import unittest
import rospy
import pathlib
import os
from mdis_state_machine import *


TEST_DIR = pathlib.Path(__file__).parent.absolute()
LAUNCH_DIR = os.path.join(TEST_DIR.parent.absolute(), 'launch')
LAUNCH_FILE = os.path.join(LAUNCH_DIR, 'test_state_machine.launch')
EXPLORER_NAME = "test_robot_1"
RELAY_NAME = "test_robot_2"
DATA_CENTER = "data_center"


class TestStateShiftExplorer(unittest.TestCase):

	def test_success_state(self: unittest.TestCase) -> None:
		proc = start_state_machine(LAUNCH_FILE)
		rospy.init_node("test_node")

		self.assertEqual(listenStateChange(GO_TO_EXPLORE, EXPLORER_NAME), True, "Initial state check for explorer failed")
		self.assertEqual(listenStateChange(GO_TO_MEET, RELAY_NAME), True, "Initial state check for relay failed")
		self.assertEqual(listenStateChange(DATA_CENTER_READY_TO_MEET, DATA_CENTER), True, "Initial state check for data center failed")

		self.assertEqual(verifyStateChange(GO_TO_EXPLORE, EXPLORE, EXPLORER_NAME), True, "State shift check for explorer state 1-2 failed")
		self.assertEqual(verifyStateChange(EXPLORE, GO_TO_MEET, EXPLORER_NAME), True, "State shift check for explorer state 2-3 failed")

		rospy.sleep(1)
		self.assertEqual(verifyConnStateChange(GO_TO_MEET, TRANSIT_TO_MEET, EXPLORER_NAME, RELAY_NAME), True, "State shift check for explorer state 3-4 failed")
		self.assertEqual(verifyConnStateChange(GO_TO_MEET, TRANSIT_TO_MEET, RELAY_NAME, EXPLORER_NAME), True, "State shift check for relay state 3-4 failed")

    ## Intermidiate state changes should happen automatically
		rospy.sleep(1)
		
		self.assertEqual(verifyStateChange(MERGE_MAP, DECIDE_NEXT_MEETING, EXPLORER_NAME), True, "State shift check for explorer state 5-6 failed")
		self.assertEqual(verifyStateChange(MERGE_MAP, RECEIVE_NEXT_MEETING, RELAY_NAME), True, "State shift check for relay state 5-7 failed")

		self.assertEqual(verifyMeetingLocCalcStateChange(DECIDE_NEXT_MEETING, END_MEETING, EXPLORER_NAME), True, "State shift check for explorer state 6-8 failed")
    ## Excavator state changes will end here; Relay will continue

    ## Intermidiate state changes should happen automatically
		rospy.sleep(1)
		self.assertEqual(verifyConnStateChange(GO_TO_DUMP_DATA, TRANSIT_TO_MEET, RELAY_NAME, DATA_CENTER), True, "State shift check for relay state 9-4 failed")

		rospy.sleep(1)
		self.assertEqual(verifyStateChange(MERGE_MAP, END_MEETING, DATA_CENTER), True, "State shift check for data center state 5-6 failed")
		self.assertEqual(verifyStateChange(MERGE_MAP, END_MEETING, RELAY_NAME), True, "State shift check for relay state 5-7 failed")

		rospy.sleep(2)

		self.assertEqual(listenStateChange(GO_TO_EXPLORE, EXPLORER_NAME), True, "Final state check for explorer failed")
		self.assertEqual(listenStateChange(GO_TO_MEET, RELAY_NAME), True, "Final state check for relay failed")
		self.assertEqual(listenStateChange(DATA_CENTER_READY_TO_MEET, DATA_CENTER), True, "Final state check for data center failed")


		stop_state_machine(proc)

if __name__ == '__main__':
	unittest.main()
