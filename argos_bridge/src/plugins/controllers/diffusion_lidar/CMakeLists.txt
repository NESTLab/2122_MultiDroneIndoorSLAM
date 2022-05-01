add_library(diffusion_lidar MODULE diffusion_lidar.h diffusion_lidar.cpp)
target_link_libraries(diffusion_lidar
  diffusion_ros
  differential_drive
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_kheperaiv
  /opt/ros/$ENV{ROS_DISTRO}/lib/libtf.so
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)