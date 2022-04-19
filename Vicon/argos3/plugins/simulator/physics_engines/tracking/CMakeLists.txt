set(UPDATER_HEADERS
  tracking_updaters/tracking_updater.h
  tracking_updaters/vicon_updater.h
  tracking_updaters/spam_updater.h
  tracking_updaters/csv_updater.h
  tracking_updaters/kilotrack_updater.h)

set(UPDATER_SOURCES
  tracking_updaters/vicon_updater.cpp
  tracking_updaters/spam_updater.cpp
  tracking_updaters/csv_updater.cpp
  tracking_updaters/kilotrack_updater.cpp)

set(HEADERS
  tracking_model.h
  tracking_khepera_model.h
  tracking_kilobot_model.h
  tracking_box_model.h
  tracking_cylinder_model.h
  tracking_engine.h)

set(SOURCES
  tracking_model.cpp
  tracking_khepera_model.cpp
  tracking_kilobot_model.cpp
  tracking_box_model.cpp
  tracking_cylinder_model.cpp
  tracking_engine.cpp
)

add_library(argos3plugins_simulator_tracking SHARED ${HEADERS} ${SOURCES} ${UPDATER_HEADERS} ${UPDATER_SOURCES})

target_link_libraries(argos3plugins_simulator_tracking
  ViconDataStreamSDK_CPP
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_entities
  argos3plugin_simulator_genericrobot
  argos3plugins_controller_assigner
  argos3plugins_simulator_datatypes
  argos3plugins_simulator_moveable_tcp_socket
)

install(TARGETS argos3plugins_simulator_tracking DESTINATION lib/argos3)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}")
install(FILES ${UPDATER_HEADERS} DESTINATION "include/${relative_dir}/tracking_updaters")
