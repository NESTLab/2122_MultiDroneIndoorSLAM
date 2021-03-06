[![Build Status](https://app.travis-ci.com/NESTLab/2122_MultiDroneIndoorSLAM.svg?branch=main)](https://app.travis-ci.com/NESTLab/2122_MultiDroneIndoorSLAM)

# 2122_MultiDroneIndoorSLAM
Code for 21/22 MQP and DRs on Multi Drone Indoor SLAM

# Quick Start
For rapid development, you can build this project in a container. This requires Docker Compose, which can be installed [here](https://docs.docker.com/compose/install/).

> IMPORTANT: Running this project inside a container will significantly contrain performance. These images are designed for development and demo perposes only.


```zsh
# From within the project's root directory
docker-compose up
```
This builds a custom image based on ubuntu LTS along with all the software required for this project, such as: ROS Noetic, ARGos, Gazebo etc.

Interact with the container's Full Desktop Environment via browser at [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html)

![Docker dev container's desktop environment](desktop-env-screenshot.png)

Or attach a shell session:
```zsh
docker exec -it 2122_multidroneindoorslam_novnc_1 /bin/bash
```


> If you see an error `2122_multidroneindoorslam-novnc-1  | standard_init_linux.go:228: exec user process caused: no such file or directory` (possibly while running on Windows)

> Change the file `entrypoint.sh` end of line conversion from CRLF to LF

# Local Setup
> NOTE: We are using Ubuntu 20.04 for this example
1. Follow the instructions for installing [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Gazebo](http://www.gazebosim.org/tutorials?tut=install_ubuntu)

2. Install project dependencies: [turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) and [dwa_local_planner](http://wiki.ros.org/dwa_local_planner)

```zsh
sudo apt install ros-noetic-turtlebot3 ros-noetic-dwa-local-planner 
```

3. Create a workspace and clone the repository
```bash
# Create a workspace
# (not needed if you already have a working workspace)
mkdir -p ~/catkin_ws/src 

# Clone the shared repository in the src directory
# Cloning with an option --recurse-submodules is necessary, as we have some git submodules
cd ~/catkin_ws/src 
git clone --recurse-submodules https://github.com/NESTLab/2122_MultiDroneIndoorSLAM.git

# Install local dependencies
rosdep install --from-paths ./ --rosdistro noetic -y

# Compile the repository
cd ~/catkin_ws
catkin_make

# Add the sourcing of workspace in bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Specify the turtlebot3 model you'd like to use
# options: burger | waffel | waffle_pi
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```
## Test run turtlebot in dan's huge maze
* Test if the turtlebot3 launches all right with the command:
* It should launch gazebo sim of the Dan's maze with one robot in it
* It should also start to explore on its own
```bash
roslaunch turtlebot3_gazebo multi_turtlebot3_all.launch 
```

>These instructions worked on my (Ashay's) PC. Let me know if anything needs to be changed


## Instructions


#### Make Ros Package
```shell
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cp -r multi_map_ros ~/catkin_ws/src/
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

#### Startup Ros (In The Background)
```shell
$ roscore 2>&1 &
```

#### Run a package
```shell
$ rosrun <package> <node_file_name>
$ rosrun multi_map_ros multi_map_node.py
```

### Debugging
```
rosnode # list | show | info ...

rosmsg # list | show | info ...

rosrun [--prefix cmd] [--debug] PACKAGE EXECUTABLE [ARGS]

rostopic echo <pub or sub name>
```

