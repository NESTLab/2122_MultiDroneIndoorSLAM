set(HEADERS
	khepera_pheromone_layer.h
	khepera_pheromone_follower.h)

set(SOURCES
	khepera_pheromone_layer.cpp
	khepera_pheromone_follower.cpp)

add_library(khepera_pheromone SHARED ${HEADERS} ${SOURCES})

target_link_libraries(khepera_pheromone
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_genericrobot
  khepera_diffusion
  argos3plugin_simulator_generic_pheromone
)

install(TARGETS khepera_pheromone DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "${CMAKE_BINARY_DIR}/include/${relative_dir}")
