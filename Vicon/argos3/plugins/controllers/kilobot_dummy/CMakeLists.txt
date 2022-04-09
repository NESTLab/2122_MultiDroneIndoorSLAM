add_library(kilobot_dummy SHARED kilobot_dummy.h kilobot_dummy.cpp)
target_link_libraries(kilobot_dummy
  argos3core_simulator
  argos3plugin_simulator_kilobot)

install(TARGETS kilobot_dummy DESTINATION lib)
  