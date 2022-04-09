set(HEADERS
  pheromone_medium.h)

set(SOURCES
  pheromone_medium.cpp)

add_library(pheromone_medium SHARED ${HEADERS} ${SOURCES})

target_link_libraries(pheromone_medium
  argos3core_simulator)

install(TARGETS pheromone_medium DESTINATION ${CMAKE_BINARY_DIR}/lib)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "${CMAKE_BINARY_DIR}/include/${relative_dir}")
