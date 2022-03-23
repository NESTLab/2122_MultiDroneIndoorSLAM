SHELL=bash
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
PATH_TO_MAKEFILE=$(abspath $(lastword $(MAKEFILE_LIST)))
WORKDIR=$(shell dirname $(PATH_TO_MAKEFILE))

c:
	$(SETUP); \
	python3 $(WORKDIR)/coms/coms/src/mapmerge/get_maps.py

install-state-machine:
	pip install -e $(WORKDIR)/mdis_state_machine

install: install-state-machine
	$(SETUP); \
	$(MAKE) -C $(WORKDIR)/coms install

test-state-machine: test-state-machine-explorer test-state-machine-relay

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

argos-demo:
	$(SETUP); \
	argos3 -c $(WORKDIR)/coms/argos_bridge/argos_worlds/multi_robot_dan_maze.argos


gazebo-net-sim:
	$(SETUP); \
	roslaunch example gazebo.launch

argos-net-sim:
	$(SETUP); \
	roslaunch example argos.launch

coms-net:
	$(SETUP); \
	roslaunch coms net.launch

teleop-0:
	$(SETUP); \
	roslaunch turtlebot3_teleop demo.launch
