set(HEADERS
  controller_assigner.h
  controller_rule.h
  controller_rule_limited.h
  controller_rule_unlimited.h
  controller_rule_default.h)

set(SOURCES
  controller_assigner.cpp
  controller_rule_limited.cpp
  controller_rule_unlimited.cpp
  controller_rule_default.cpp)

add_library(argos3plugins_controller_assigner SHARED ${HEADERS} ${SOURCES})

target_link_libraries(argos3plugins_controller_assigner
  argos3core_simulator)

install(TARGETS argos3plugins_controller_assigner DESTINATION lib/argos3)

FILE(RELATIVE_PATH relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
install(FILES ${HEADERS} DESTINATION "include/${relative_dir}")
