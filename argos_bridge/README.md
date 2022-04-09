Introduction
------------

The purpose of this package is to connect ROS with the ARGoS (henceforth argos)
robot simulator.  

    http://www.argos-sim.info

The basic concept is the use of an argos plugin (in the 'plugin' directory).
When argos is executed it looks at ARGOS_PLUGIN_PATH which you should have
configured to find this plugin.  The plugin represents a single robot which
subscribes and publishes to ROS topics.  If the argos world has multiple robots,
they will each have a separate instance of the C++ class in the 'plugin' dir
which connects to ROS.

Setup
-----
Argos bridge is provided with a docker environment which includes all components needed for a simple demo.  
Start the container using:  

    docker-compose up --build  

Open a bash shell using:  

    docker exec -it $(docker ps --format "{{.ID}}") /bin/bash

Use
---
Find the launch script at argos_brige/launch to start a simulation.  
Example usage:  

    ./launch.sh [name of argos world] [name of ros launch file]

Store argos worlds and ros launch files in the launch directory according to the file structure:

    /launch
      |-> /argos_worlds {store argos files here}
      |-> /roslaunch_files {store ros launch files here}
      |-> launch.sh {launch script for simulations}
      |-> killall.sh {kills all ros and argos processes to end a simulation}

File Structure
------
    /argos_brige
      |-> CMakeLists.txt    
      |-> package.xml       
      |-> /launch           
            |-> /argos_worlds
            |-> /roslaunch_files
      |-> /msg              
      |-> /plugin            
            |-> CmakeLists.txt
            |-> /loop_functions
                  |-> /clock_loop_functions
            |-> /controllers
                  |-> /khepraiv_ros  
                  |-> /abstract_ros_controller
      |-> ros_lib_links
      |-> scripts

Issues
------

Author
------
Peter Nikopoulos (peter@nikopoulos.net)  
(originally forked from BOTSlab/argos_bridge)