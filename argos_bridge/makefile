SHELL=bash
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
COVERALLS_REPO_TOKEN=x914kIwyMzM7CLf2ixlx0xwW84IF8tUPT
PATH_TO_MAKEFILE=$(abspath $(lastword $(MAKEFILE_LIST)))
WORKDIR=$(shell dirname $(PATH_TO_MAKEFILE))

check-health:
	source $(WORKDIR)/health-check.sh

install-deps:
	pip install -r $(WORKDIR)/coms/requirements-dev.txt; \
	pip install -r $(WORKDIR)/coms/requirements.txt

install-coms: install-deps; \
	pip install -e $(WORKDIR)/coms

