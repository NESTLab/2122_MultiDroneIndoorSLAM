set(HEADERS
  network_loop_function.h)

set(SOURCES
  network_loop_function.cpp)


add_library(argos3plugin_simulator_tracking_networking SHARED ${HEADERS} ${SOURCES})

target_link_libraries(argos3plugin_simulator_tracking_networking
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_entities
  argos3plugin_simulator_genericrobot
  argos3plugins_simulator_moveable_tcp_socket
  argos3plugins_simulator_tracking
  ViconDataStreamSDK_CPP)
  
install(TARGETS argos3plugin_simulator_tracking_networking DESTINATION lib/argos3)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}")
