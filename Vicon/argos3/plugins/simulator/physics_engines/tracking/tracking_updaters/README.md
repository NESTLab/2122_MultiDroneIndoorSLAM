# Tracking Updaters

Tracking updaters allow for the Tracking engine to use position and orientation data from sources other than Argos itself. Every step of the experiment the Update method of all active tracking updaters will be called. This method returns a vector TRobotInfo. Each element in the vector corresponds to a robot that is to be updated and provides information about the robot type, name, position, and orientation. There is also a field for providing extra information to the tracking updater and is currently unused.

## Creating a Tracking Updater

All Tracking Updater classes inherit from the CTrackingUpdater class which provides the Init, Update, Reset, Destroy methods for the inheriting class to override. More information can be found in the tracking_updater.h file.

Each tracking updater must register the class by invoking the REGISTER_TRACKING_UPDATER(CLASSNAME, LABEL) macro. This is normally done in the implementation .cpp file of the  class.
