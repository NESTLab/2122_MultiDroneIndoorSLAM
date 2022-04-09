set(ENV{BUZZ_INCLUDE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

buzz_make(test.bzz INCLUDES 	string.bzz 
								vec2.bzz
								diffusion.bzz  
								persistant_coverage.bzz)

buzz_make(vsk-known.bzz INCLUDES 	string.bzz)

install(FILES 
	${CMAKE_CURRENT_BINARY_DIR}/test.bo 
	${CMAKE_CURRENT_BINARY_DIR}/test.bdb 
	${CMAKE_CURRENT_BINARY_DIR}/vsk-known.bo 
	${CMAKE_CURRENT_BINARY_DIR}/vsk-known.bdb
	DESTINATION ${CMAKE_BINARY_DIR}/lib)
