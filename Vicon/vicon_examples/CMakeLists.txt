add_executable(Vicon_Example 
	vicon.cpp vicon.h
	vicon_driver.cpp vicon_driver.h)

target_link_libraries(Vicon_Example 
	ViconDataStreamSDK_CPP
	pthread)
