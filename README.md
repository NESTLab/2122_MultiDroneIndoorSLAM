# 2122_MultiDroneIndoorSLAM
Code for 21/22 MQP and DRs on Multi Drone Indoor SLAM

# Setup Instructions
1. You will need Ubuntu 20.04 and ROS Noetic to get this repository working (You can install with instructions at [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) )
1. Install packages needed for [turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) (Not sure if this is needed, but I did it and don't know if the sim would run without it)
1. Create a workspace and clone the repository
```bash
# Create a workspace
# (not needed if you already have a working workspace)
mkdir -p ~/catkin_ws/src 

# Clone the shared repository in the src directory
# Cloning with an option --recurse-submodules is necessary, as we have some git submodules
cd ~/catkin_ws/src 
git clone --recurse-submodules https://github.com/NESTLab/2122_MultiDroneIndoorSLAM.git

# Compile the repository
cd ~/catkin_ws
catkin_make

# Add the sourcing of workspace in bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Specify the turtlebot3 model you'd like to use
# I am using waffle
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