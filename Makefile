SHELL=bash
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
PATH_TO_MAKEFILE=$(abspath $(lastword $(MAKEFILE_LIST)))
WORKDIR=$(shell dirname $(PATH_TO_MAKEFILE))

test-coms:
	$(SETUP); \
	$(MAKE) -C $(WORKDIR)/coms install-coms test

test: test-coms

sim-gazebo:
	$(SETUP); \
	roslaunch turtlebot3_gazebo multi_turtlebot3_all.launch

check-health:
	source $(WORKDIR)/health-check.sh