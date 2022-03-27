INSTRUCTIONS:

To get the image and start the container:

docker pull tberg1234/2122_multidroneindoorslam:latest

docker-compose -f docker-compose-image.yaml up

To get into the container:

docker exec -it 2122_multidroneindoorslam_novnc_1 /bin/bash

Once inside container open 3 shells using the above command and run:

    1. roslaunch a3c_turtlebot3 setup.launch
    2. rosrun multi_map_ros move_base_reset.py
    3. roslaunch a3c_turtlebot3 start_a2c_training.launch

Once you run the last command the training will begin.  You can view the progress in the web browser (http://localhost:8080/vnc.html), but you can know it is done when the final (3rd) roslaunch finishes cleanly and returns.  

To run trained model on one robo inclusing explorationt:

    1. roslaunch a3c_turtlebot3 setup.launch
    2. rosrun multi_map_ros move_base_reset.py
    3. roslaunch a3c_turtlebot3 start_trained_a2c_v3.launch
    
To run trained model with publishing frontiers for state machine integration:

rosrun a3c_turtlebot3 a2c_v3_trained_for_state_machine.py 
roslaunch a3c_turtlebot3 full_start_trained_a2c_v3_for_state_machine.launch

MAKE SURE TO DOWNLOAD MODEL FILES WHICH ARE TOO LARGE FOR GIT - in our drive under ML_Model


