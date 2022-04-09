set(HEADERS
  diffusion_visualizer.h)

set(SOURCES
  diffusion_visualizer.cpp)

add_library(diffusion_visualizer SHARED ${HEADERS} ${SOURCES})

target_link_libraries(diffusion_visualizer
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_qtopengl
  argos3plugin_simulator_kheperaiv
  khepera_diffusion
  ${ARGOS_QTOPENGL_LIBRARIES})

install(TARGETS diffusion_visualizer DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "${CMAKE_BINARY_DIR}/include/${relative_dir}")
