set(HEADERS
  moveable_tcp_socket.h)

set(SOURCES
  moveable_tcp_socket.cpp)

add_library(argos3plugins_simulator_moveable_tcp_socket SHARED ${HEADERS} ${SOURCES})

target_link_libraries(argos3plugins_simulator_moveable_tcp_socket
  argos3core_simulator)

install(TARGETS argos3plugins_simulator_moveable_tcp_socket DESTINATION lib/argos3)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}")
