SHELL=bash
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
PATH_TO_MAKEFILE=$(abspath $(lastword $(MAKEFILE_LIST)))
WORKDIR=$(shell dirname $(PATH_TO_MAKEFILE))

install-state-machine:
	pip install -e $(WORKDIR)/mdis_state_machine

test-state-machine: install-state-machine
	$(SETUP); \
	python3 -m unittest discover $(WORKDIR)/mdis_state_machine/tests

test-state-machine-explorer: install-state-machine
	$(SETUP); \
	python3 -m unittest $(WORKDIR)/mdis_state_machine/tests/test_state_shift_explorer.py

test-state-machine-relay: install-state-machine
	$(SETUP); \
	python3 -m unittest $(WORKDIR)/mdis_state_machine/tests/test_state_shift_relay.py

test-coms:
	$(SETUP); \
	$(MAKE) -C $(WORKDIR)/coms install-coms test

test: test-coms test-state-machine

sim-gazebo:
	$(SETUP); \
	roslaunch turtlebot3_gazebo multi_turtlebot3_all.launch

check-health:
	source $(WORKDIR)/health-check.sh