# Overview

This main feature this plugin provides is the Tracking Engine. This physics engine allows for updates to the physics system to be provided from an external source, such as the Vicon motion capture system.

Additional features include the Master Loop Function that allows for multiple userfunctions, and controller assiners that allow dynamic allocations of robot controllers for robots that are added to the experiement at any time.

# Dependencies
This plugin depends on the following:

* Argos3 Simulator
* Buzz Programming Language
* Khepera and Kilobot plugins for argos3 

These dependencies must be installed in the listed order to use this library in the fullest.


# Compilation Installation
After installing the dependencies and cloning this repo simply run:

```
mkdir build
cd build
cmake ..
make
sudo make install
```

# Argos Configuration

# Tracking engine
The core of this library, the tracking engine allows for external methods for updating argos states. This engine allows for robots to be added while a experiement is executing.

Currently buzz requires that one robot of each type with a buzz controller be present. 

If the Range and Bearing medium is in use you can set the range and data size in the tracking node of the argos configurations file, see the example configuration below.

## Modules

### Updaters
Updaters provide information for the tracking engine so that it can update the robots. Multiple updaters can be used simultaneously, but only one updater should provide information for one robot. 

If an updater does not provide an update for a robot, the robot is not updated and the previous values are used. 

#### vicon updater
Takes data from the Vicon motion capture system using its Datastream API. You must create create each Vicon model in the Vicon software that supports data stream API (Tested using Vicon Tracker).

You can set the ip or hostname via the host attribute and the port via the port attribute. The example below will attempt to connect to a Vicon DataStream sever at "192.168.0.1:801".

```xml
  <vicon_updater host="192.168.1.211" port="801">
``` 

#### csv updater
Reads data from a csv file, the first line is the name of each robot, while the following lines contain the pose data for the robots. In each line the robot data is seperated by a ','. The pose data contains the XYZ cordinates and the Quaternion (WXYZ) for the robot seperated by ' 's.

You can set the file location via the file attribute. The example below will attempt to open and parse the file "location_data.csv".

```xml
  <csv_updater file="location_data.csv">
``` 

#### spam updater
The spam updater is main used for testing and simply adds a new robot of the given type for each frame. This is repeated up to the limit, if one is provided via the limit attribute. The type of robot is denoted by the robot attribute. The exampe bellow will create 5 khepera robots in 5 frames.

```xml
  <spam_updater robot="Khepera" limit="5"/> 
```

#### kilotrack
KiloTrack is meant to be used with the KiloTrack system, a tracking system specifically made for the NESTLab table. You can provide the port and host of the system runing the KiloTrack system.

You can set the ip or hostname via the host attribute and the port via the port attribute. The example below will attempt to connect to a Vicon DataStream sever at "192.168.0.1:801".

```xml
  <kilotrack host="192.168.0.1" port="801">
``` 

### Controller Assigners
Controller assigners are how controllers for robots created on the fly are determined. The nodes are assesed in decending order based on their "priority" attribute (default 0). When a node is assed it returns a controller or passes onto the next controller.  When all assigners pass, the default controller is used. There are currently 2 types of controller assigners.

Each type of robot can have a series of assigners, this is denoted by having a tag with the name of the robot type. Each robot type must have a default controller, but may have any number of assigners. 

#### Limited
Assigns controllers until the "limit" attribute of robots have been assigned at which point the assigner passes.

#### Unlimited
Assigns all robots to the given controller, this asigner never passes.


## Sample Argos Configuration

```xml
    <physics_engines>
      <tracking>
        <updaters>
          <vicon_updater/>
          <csv_updater file="test.csv" />
          <spam_updater robot="Khepera" limit="11"/>
          <kilotrack_updater port="1337" host="localhost" />
        </updaters>
        <assigners>
          <Khepera default="layer1">
            <limited controller="layer1" limit="3" priority="2"/>
            <limited controller="layer2" limit="4" priority="1"/>
            <limited controller="layer3" limit="2" priority="0"/>
          </Khepera>
        </assigners>
        <rab>
          <Khepera data_size="1000" range="2" />
        </rab>
      </tracking>
    </physics_engines>
```

# Master Loop Function
This allows experiements to specify multiple loop functions, they can be used out of the box like any other loop function as seen below.

## Sample Argos Configuration

```xml
<loop_functions label="MasterLoopFunctions">
  <sub_loop_function label="BaseNetworking" />
  <sub_loop_function label="diffusion_loop_functions" />
</loop_functions>
```

### Accessing Loop Functions
The master loop function allows access to all of its sub-loop funcions via the static `GetLoopFunction` method, you do need to include `argos3/plugins/loop_functions/master_loop_functions/master_loop_functions.h` and link against `argos3plugin_simulator_master_loop_functions`.

If MasterLoopFunctions is not the loop function being used `GetLoopFunction` will simply return the loop function currently being used. This allows for you to use the same code whether or not the master loop function is actually present in your code. 


# Networking Loop Function
Allows for robots to connect to the tracking engine over TCP/IP. By default the loop function listens for incomming connections on port 22222 this can be chaged via the "port" attribute.

# Experiments
Currently example experiments are included in this project. These may be moved to a separate project at a later date. More information can be found [here](argos3/experiments/).
