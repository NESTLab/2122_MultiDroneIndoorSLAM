set(HEADERS
  khepera_diffusion.h)

set(SOURCES
  khepera_diffusion.cpp)

add_library(khepera_diffusion SHARED ${HEADERS} ${SOURCES})

target_link_libraries(khepera_diffusion
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_genericrobot)

install(TARGETS khepera_diffusion DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "${CMAKE_BINARY_DIR}/include/${relative_dir}")
  