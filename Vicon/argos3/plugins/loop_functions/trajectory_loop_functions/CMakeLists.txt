set(HEADERS
  trajectory_loop_functions.h
  trajectory_qtuser_functions.h)

set(SOURCES
  trajectory_loop_functions.cpp
  trajectory_qtuser_functions.cpp)

add_library(trajectory_loop_functions SHARED ${HEADERS} ${SOURCES})

target_link_libraries(trajectory_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_qtopengl
  argos3plugin_simulator_master_loop_functions
  ${ARGOS_QTOPENGL_LIBRARIES})

install(TARGETS trajectory_loop_functions DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}")

