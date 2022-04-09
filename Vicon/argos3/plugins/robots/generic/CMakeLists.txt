set(HEADERS_1
	control_interface/ci_pheromone_sensor.h
	control_interface/ci_pheromone_actuator.h)

set(HEADERS_2
	simulator/pheromone_default_sensor.h
	simulator/pheromone_default_actuator.h)

set(SOURCES
	control_interface/ci_pheromone_sensor.cpp
	control_interface/ci_pheromone_actuator.cpp
	simulator/pheromone_default_sensor.cpp
	simulator/pheromone_default_actuator.cpp)

add_library(argos3plugin_simulator_generic_pheromone SHARED ${HEADERS} ${SOURCES})

target_link_libraries(argos3plugin_simulator_generic_pheromone
  argos3core_simulator
  pheromone_medium)

install(TARGETS argos3plugin_simulator_generic_pheromone DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS_1} DESTINATION "${CMAKE_BINARY_DIR}/include/control_interface")
install(FILES ${HEADERS_2} DESTINATION "${CMAKE_BINARY_DIR}/include/simulator")
