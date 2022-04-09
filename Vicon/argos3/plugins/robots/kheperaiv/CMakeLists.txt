set(HEADERS
  simulator/kheperaiv_battery_tracked_sensor.h
  simulator/kheperaiv_proximity_tracked_sensor.h
  simulator/kheperaiv_proximity_st_combined_sensor.h)

set(SOURCES
  simulator/kheperaiv_battery_tracked_sensor.cpp
  simulator/kheperaiv_proximity_tracked_sensor.cpp
  simulator/kheperaiv_proximity_st_combined_sensor.cpp)

add_library(argos3plugins_simulator_tracked_kheperaiv SHARED ${HEADERS} ${SOURCES})

target_link_libraries(argos3plugins_simulator_tracked_kheperaiv
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugins_simulator_tracking
)

install(TARGETS argos3plugins_simulator_tracked_kheperaiv DESTINATION lib/argos3)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}/simulator")
