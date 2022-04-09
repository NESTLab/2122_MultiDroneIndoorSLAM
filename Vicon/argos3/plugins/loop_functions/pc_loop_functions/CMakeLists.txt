set(HEADERS
  pc_loop_functions.h
  pc_qt_user_functions.h)

set(SOURCES
  pc_loop_functions.cpp
  pc_qt_user_functions.cpp)

add_library(pc_loop_functions SHARED ${HEADERS} ${SOURCES})

target_link_libraries(pc_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_buzz
  ${BUZZ_LIBRARY}
  argos3plugin_simulator_qtopengl
  argos3plugin_simulator_master_loop_functions
  argos3plugin_simulator_kheperaiv
  ${ARGOS_QTOPENGL_LIBRARIES})

install(TARGETS pc_loop_functions DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "${CMAKE_BINARY_DIR}/include/${relative_dir}")
