FROM ubuntu:focal

# Setup environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes

# Install git, supervisor, VNC, & X11 packages
RUN set -ex; \
    apt-get update; \
    apt-get install -y \
      sudo \
      lsb-core \
      gnupg2 \
      curl \
      bash \
      fluxbox \
      git \
      net-tools \
      novnc \
      supervisor \
      x11vnc \
      xterm \
      xvfb \
      python3-pip; \
    git clone https://github.com/theasp/docker-novnc.git /app;

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'; \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -; \
    apt-get update; \
    apt-get install ros-noetic-desktop-full -y; \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc; \
    apt-get install -y python3-rosdep python3-rosinstall ros-noetic-turtlebot3 ros-noetic-dwa-local-planner ros-noetic-gmapping ros-noetic-rviz python3-rosinstall-generator python3-wstool build-essential python3-rosdep;

# Build ARGos3 from source
RUN apt-get install -y libgsl-dev cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev \
  lua5.3 doxygen graphviz libgraphviz-dev asciidoc; \
  git clone https://github.com/ilpincy/argos3.git; \
  cd argos3; \
  mkdir build_simulator; \
  cd build_simulator; \
  cmake -DCMAKE_BUILD_TYPE=debug ../src; \
  make; \
  make doc; \
  echo '/usr/local/lib' >> /etc/ld.so.conf; \
  echo "sudo ldconfig" >> ~/.bashrc; \
  make install

# Install kheperaiv robot to ARGos3
RUN git clone https://github.com/ilpincy/argos3-kheperaiv.git; \
  cd argos3-kheperaiv && mkdir build_sim && cd build_sim; \
  cmake -DCMAKE_BUILD_TYPE=debug ../src; \
  make; \
  sudo make install

# # Install Kilobot robot
# RUN git clone https://github.com/ilpincy/argos3-kilobot.git; \
#   cd argos3-kilobot && mkdir build && cd build; \
#   cmake -DCMAKE_BUILD_TYPE=Release ../src; \
#   make; \
#   sudo make install

# # Install BUZZ Language
# RUN git clone https://github.com/buzz-lang/Buzz.git buzz; \
#   mkdir buzz/build && cd buzz/build; \
#   cmake -DCMAKE_BUILD_TYPE=Release ../src; \
#   make; \
#   sudo make install; \
#   sudo ldconfig

# # Install the Vicon [Platform Specific: Linux x86-64]
# COPY ./Vicon /root/vicon

# RUN cp /root/vicon/vicon_sdk/Linux64/libViconDataStreamSDK_CPP.so /lib/libViconDataStreamSDK_CPP.so; \
#   cp /root/vicon/vicon_sdk/Linux64/libboost_system-mt.so.1.58.0 /lib/libboost_system-mt.so.1.58.0; \
#   cp /root/vicon/vicon_sdk/Linux64/libboost_thread-mt.so.1.58.0 /lib/libboost_thread-mt.so.1.58.0; \
#   mkdir /root/vicon/build && cd /root/vicon/build; \
#   cmake ..; \
#   make; \
#   sudo make install

# Install extra dependencies
RUN sudo apt-get update; \
  sudo apt-get upgrade -y; \
  sudo apt-get install -y \
    python-is-python3 \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    iproute2 \
    ros-noetic-catkin \
    python3-catkin-tools \
    libtf2-ros-dev

# Add ROS dependent scripts
RUN rosdep init; \
  rosdep update; \
  mkdir -p /root/catkin_ws/src; \
  echo "cd /root/catkin_ws;" >> ~/.bashrc; \
  echo "source /root/catkin_ws/devel/setup.bash;" >> ~/.bashrc; \
  echo "export TURTLEBOT3_MODEL=burger"; \
  echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/opt/ros/noetic/lib" >> ~/.bashrc; \
  echo "export ARGOS_PLUGIN_PATH=$HOME/catkin_ws/src/argos_bridge/ros_lib_links" >> ~/.bashrc; \
  echo "export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:$HOME/catkin_ws/devel/lib" >> ~/.bashrc; 

# add ros packages 
RUN git clone -b noetic-devel https://github.com/hrnr/m-explore.git; \
  git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git; \
  cd /root/catkin_ws/src; \
  ln -s /m-explore .; \
  cd /root/catkin_ws/src; \
  ln -s /turtlebot3_simulations/ .; \ 
  sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan \
  ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
  ros-noetic-rosserial-server ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* \
  ros-noetic-frontier-exploration ros-noetic-navigation-stage \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers; \
  sudo apt-get install ros-noetic-dynamixel-sdk; \
  sudo apt-get install ros-noetic-turtlebot3-msgs; \
  sudo apt-get install ros-noetic-turtlebot3;
EXPOSE 8080

CMD ["/root/catkin_ws/src/entrypoint.sh"]