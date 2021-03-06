### START - Adapted from argos3-examples/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.12)
project(argos3_bridge)

# enable build arguments: TODO
set(BUILD_TESTS TRUE)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})
string(REPLACE "/lib/argos3" "" ARGOS_PREFIX "${ARGOS_LIBRARY_DIR}")
set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install directories." FORCE)
# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
# include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
find_package(Lua REQUIRED)

# Set ARGoS include dir
include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

add_subdirectory(plugins)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(BUILD_TESTS)

### END