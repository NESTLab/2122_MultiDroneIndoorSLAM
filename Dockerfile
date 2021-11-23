FROM ros:noetic
# For running bash scripts 
SHELL ["/bin/bash", "-c"]
# Setup work environment
WORKDIR /catkin_ws
COPY ./multi_map_ros /catkin_ws/src
COPY ./startup.bash /startup.bash
# Start roscore in the background
CMD ["/startup.bash"]
