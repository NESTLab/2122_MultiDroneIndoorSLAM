[![Build Status](https://app.travis-ci.com/NESTLab/coms.svg?branch=main)](https://app.travis-ci.com/NESTLab/coms)
[![Coverage Status](https://coveralls.io/repos/github/NESTLab/coms/badge.svg?branch=HEAD)](https://coveralls.io/github/NESTLab/coms?branch=HEAD)

# Coms 📡
Distributed Communication Protocol for Robots running [ROS1](https://www.ros.org).

## Overview 🔍
Coms acts as a middleman between all network traffic, regardless of the deployment environment. It takes care of all robot-to-robot communications, whether they are deployed on raspberry pis, or run within a simulation environment such as Gazebo. This package enables your ROS nodes to rely on a standard communication protocol for external message passing. In essence, Coms works as a ROS1 implementation of a multi-master approach for orchestrating distributed ad-hoc networks.

## ROS Topics 💬
For simulations, the Sim._broadcaster sends a `nearby.msg` to the `/nearby_robots` topic whenever it sees another robot in line of site. Different physics simulations, such as ARGos and Gazebo, have their own way of determining line-of-site. Based on the configuration run by
the `ros-net-sim` package, a given physics_sim may be invoked to match the robot's environment.

ARGos3 - Line-of-site topics are generated for each robot run within its physics engine.
The broadcaster subscribes to a given `/robot<num>/line-of-site` topic and re-broadcasts them on a regular interval to the `/nearby_robots` topic for other ROS nodes.

## Debugging 🔧
We expose the topic: `/tb3_0/coms_debug` to see what's going on in the coms node. You can see sync and ping requests from neighbors and watch connections open & close. If you send a trigger to the topic: `/tb3_0/coms_listening`, you can see that too!

```zsh
# Start the simulation
roslaunch turtlebot3_gazebo multi_turtlebot3_all.launch
```

```zsh
# Listen to the coms debugging channel 
rostopic echo /tb3_0/coms_debug
```

The above topic will show you all incoming, outgoing, and error messages within the coms node.

```zsh
# Ask coms to invoke a sync
# NOTE: tb3_0 is bound to 192.168.0.1
#       tb3_1 is bound to 192.168.0.2
rostopic pub /tb3_0/coms_listening std_msgs/String "sync|192.168.0.2|explorer"
```

The result:
```log
data: "Recieving message from topic /tb3_0/coms_listening: sync|192.168.0.2| to sync [TOPIC]"
---
data: "Server 192.168.0.1 connected to 192.168.0.2 [ID #31 OPEN]"
---
data: "Server 192.168.0.1 connected to 192.168.0.2 [ID #32 OPEN]"
---
.
.
.
---
data: "Server 192.168.0.1 connected to 192.168.0.2 [ID #36 OPEN]"
---
data: "Synchronizer merged with neighbor 192.168.0.2 [SUCCESS]" <--------------- WIN

```

# Message Specs

## Sync 🔁
About: Attempt an on-the-fly syncronization with a neighbor
### Triggers(s)
Topic: `/tb3_X/coms_listening`
```zsh
rostopic pub /tb3_0/coms_listening std_msgs/String "sync|192.168.0.2|role"
```

## Next Meeting 🤝
About: Explorers tell relays where the next meeting will be. Relays either accept or deny the request.
Message: `coms/coms/srv/NextMeeting.srv`:
```
string robot_ip
int64[] point
float64 time
---
bool accepted
```

### Trigger(s)
Service: `/tb3_X/send_next_meeting`
```zsh
rosservice call /tb3_0/send_next_meeting {"robot_ip: '192.168.0.2', point: [23,9,1], time: 0.0"}
```


## Ready 🚦
About: Ask a neighbor if they are ready to start or stop a meeting.
Message: `coms/coms/srv/ReadyToMeet.srv`:
```
# ReadyToMeet.srv
string robot_ip
bool status
---
bool is_ready
```
### Trigger(s)

Service: `/tb3_X/ready_to_meet`
```zsh
rosservice call /tb3_0/ready_to_meet "robot_ip: '192.168.0.2'
status: false"
```

Topic: `/tb3_X/coms_listening`
```zsh
rostopic pub /tb3_0/coms_listening std_msgs/String "ready|remote_ip|status"
```
> NOTE: This will not change any state, but only probe the other robot in the the logs.

## Info ℹ
About: Ask neighbor for all network information
Message: `coms/coms/srv/NetInfo.srv`
```
string remote_ip
string state
string parent_ip
string child_ip
string role
---
string ip
string state
string parent_ip
string child_ip
string role
```

### Trigger(s)

Service: `/tb3_X/ready_to_meet`
```zsh
rosservice call /tb3_0/get_info "{remote_ip: '192.168.0.2',
state: 'you_choose', parent_ip: 'you_choose', child_ip: 'you_choose', role: 'explorer'}"
```

Topic: `/tb3_X/coms_listening`
```zsh
rostopic pub /tb3_0/coms_listening std_msgs/String "info|remote_ip|state|parent_ip|child_ip|role"
```


