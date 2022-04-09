add_subdirectory(khepera_diffusion)
add_subdirectory(khepera_pheromone)
add_subdirectory(kilobot_dummy)

#message("BUZZ_FOUND:${BUZZ_FOUND}")
#if buzz is install, compile the buzz controllers
if(BUZZ_FOUND)
	add_subdirectory(buzz_controllers)
endif(BUZZ_FOUND)