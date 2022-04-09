set(HEADERS
  pheromone_visualizer.h)

set(SOURCES
  pheromone_visualizer.cpp)

add_library(pheromone_visualizer SHARED ${HEADERS} ${SOURCES})

target_link_libraries(pheromone_visualizer
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES}
  pheromone_medium
  khepera_pheromone
  argos3plugin_simulator_master_loop_functions
  diffusion_visualizer)

install(TARGETS pheromone_visualizer DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "${CMAKE_BINARY_DIR}/include/${relative_dir}")
