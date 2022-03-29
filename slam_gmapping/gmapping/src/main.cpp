/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */

#include <ros/ros.h>

#include "slam_gmapping.h"
#include "std_msgs/String.h"
bool shouldReset = false;


void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    ROS_INFO("GOT A SYS COMMAND!!!");
//    ROS_INFO("I heard: [%s]", sys_cmd->data.c_str());
//    printf("%s", sys_cmd.c_str());
    if (sys_cmd.data == "reset")
    {
      ROS_WARN("RESETTING!!!!!!");
      shouldReset = true;
    }
    else{
        ROS_WARN("NOT RESETTING");
    }
  }


  void main_loop() {

    ROS_INFO("HERE");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/syscommand", 10, sysCmdCallback);

    while (ros::ok()) {
//      ros::Duration(3).sleep();
      ROS_INFO("Back at top of mapping loop!!!");
//      std::cout << ("Back at top of mapping loop");
        {
//            ros::Duration(3).sleep();
            SlamGMapping gn;
            gn.startLiveSlam();

            while (!shouldReset) {
                ros::spinOnce();
                ros::Duration(0.2).sleep();
            }
            ROS_INFO("Resetting map...");
            shouldReset = false;
            ROS_INFO("After should reset");
        }
//        ROS_INFO("Out of scope");
//      ros::Duration(3).sleep();
    }
    ROS_INFO("ROS is no longer okay :(");
  }

  int
  main(int argc, char** argv)
  {
    ros::init(argc, argv, "slam_gmapping");

    // ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe<std_msgs::String>("/syscommand", 10, sysCmdCallback);
    main_loop();
//    ros::spin();

    return(0);
  }
