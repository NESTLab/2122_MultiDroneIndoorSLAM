add_library(clock_loop_functions MODULE
  clock_loop_function.h
  clock_loop_function.cpp
  )

target_link_libraries(clock_loop_functions
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  /opt/ros/$ENV{ROS_DISTRO}/lib/libtf.so
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
  )

  link_directories(${CMAKE_BINARY_DIR}/controllers/kheperaiv_ros)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  add_library(id_qtuser_functions MODULE 
    id_qtuser_functions.h
    id_qtuser_functions.cpp)
  
  target_link_libraries(id_qtuser_functions
    kheperaiv_ros
    argos3core_simulator
    argos3plugin_simulator_entities
    argos3plugin_simulator_kheperaiv
    argos3plugin_simulator_qtopengl
    ${ARGOS_QTOPENGL_LIBRARIES})