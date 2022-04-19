set(HEADERS
  reservable_buffer.h)

set(SOURCES
	reservable_buffer.cpp)

add_library(argos3plugins_simulator_datatypes SHARED ${HEADERS} ${SOURCES})

target_link_libraries(argos3plugins_simulator_datatypes
  argos3core_simulator)

install(TARGETS argos3plugins_simulator_datatypes DESTINATION lib/argos3)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}")
