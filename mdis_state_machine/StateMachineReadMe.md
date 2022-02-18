# Instructions to test the scheduler/state machine

## To Get started:

### Background needed:
* No additional node required
* `roscore` NOT needed

### Command to start
```bash
roslaunch mdis_state_machine test_state_machine.launch
```

## State machine summary:
* Each robot runs its own state machine, only communicating with other robots when needed
* Each state will continue unless its `transition` is triggered
___
## States flow:

* Currently, there is no 'interrupt' for any of the states. They will simply be in their loops of the states
* Each robot starts with a role, either explorer or a relay robot
* Their inital tasks are defined as:
  * RELAY: GO_TO_MEET
  * EXPLORER: GO_TO_EXPLORE

# States' loops: Explorer:

> GO_TO_EXPLORE (1) ---> EXPLORE(2)
* In actual scenarios, will be triggered after the robot reaches the exploration frontier
* In testcases, will be triggered after 5 seconds

> EXPLORE(2) ---> GO_TO_MEET(3)
* In actual scenarios, will be triggered when the time alloted for exploration elapses
* In test cases, will be triggered after 5 seconds (time calculations require robots' locations, map and move_base working)

> GO_TO_MEET(3) ---> MEET(4)
* In actual scenarios, will be triggered when the robot is connected to the party of interest
* In test cases, the tester will have to emulate the connection by publishing the requied message over rostopic
  * Currently, topic is `<ns>/connection_check`
  * Message type is: `mdis_state_machine::Connection`
* For the testing, use the connection between follows:
  * `test_robot_1`or `test_robot_2` for depending on the robot type Explorer or Relay respectively
  * Parent `dummy_parent`
* When the line of sight is implemented, this can be changed to listen to whatever the new topic and new message type is

> MEET(4) ---> GO_TO_EXPLORE (1)
* In future, should be triggered when meeting communication is completed
* Currently in code, it is an instantenious switch
* In test cases, will be triggered after 5 seconds. 

# States' loops: Relay:
* In actual scenarios, Relay robot starts after 20 seconds (simple delay before start)
* But in test cases, it will start right away
* First state is GO_TO_MEET

> GO_TO_MEET(3) ---> MEET(4)
* Same as the transition `GO_TO_MEET(3) ---> MEET(4)` described above

> MEET(4) ---> GO_TO_DUMP_DATA(5)
* Same as the transition `MEET(4) ---> GO_TO_EXPLORE (1)` described above

> GO_TO_DUMP_DATA(5) ---> DUMP_DATA(6)
* Same as the transition `GO_TO_EXPLORE (1) ---> EXPLORE(2)` described above

> DUMP_DATA(6) ---> GO_TO_MEET(3)
* Same as the transition `MEET(4) ---> GO_TO_EXPLORE (1)` described above