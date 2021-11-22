FROM ros:noetic
# For running bash scripts 
SHELL ["/bin/bash", "-c"]
# Setup work environment
WORKDIR /catkin_ws
COPY ./multi_map_ros /catkin_ws/src
# Compile the Ros Node
RUN source /ros_entrypoint.sh
RUN source /opt/ros/noetic/setup.bash && catkin_make && source devel/setup.bash
# Run the launch file
# roslaunch multi_map_ros file.launch
