add_library(differential_drive MODULE differential_drive.h differential_drive.cpp)
target_link_libraries(differential_drive
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_kheperaiv
  /opt/ros/$ENV{ROS_DISTRO}/lib/libtf.so
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)