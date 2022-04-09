cmake_minimum_required (VERSION 2.8.12)

project (vicon)

#
# Check architecture
#
if(UNIX)
  # Either Linux or MacOSX
  if(NOT APPLE)
    # It's linux, how many bits?
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
      # Linux 32 bits
      include_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Linux32)
      link_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Linux32)
      set(VICON_PATH "vicon_sdk/Linux32")
    else(CMAKE_SIZEOF_VOID_P EQUAL 4)
      # Linux 64 bits
      include_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Linux64)
      link_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Linux64)
      set(VICON_PATH "vicon_sdk/Linux64")
    endif(CMAKE_SIZEOF_VOID_P EQUAL 4)
  else(NOT APPLE)
    # MacOSX
    include_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Mac)
    link_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Mac)
    set(VICON_PATH "vicon_sdk/Mac")
  endif(NOT APPLE)
endif(UNIX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

#
# Check ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})
string(REPLACE "/lib/argos3" "" ARGOS_PREFIX "${ARGOS_LIBRARY_DIR}")
set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install directories." FORCE)

#
# Add repository directory to the list of includes
# This makes it possible to say #include <argos3/...> with
# repository includes.
#
include_directories(${CMAKE_SOURCE_DIR})

# add buzz to the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/share/buzz/cmake)

# Look for Buzz tools, libraries, and headers
find_package(Buzz)
if(BUZZ_FOUND)
  # Define Buzz-related commands
  include(UseBuzz)
  include_directories(${BUZZ_C_INCLUDE_DIR})
endif(BUZZ_FOUND)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/vicon_sdk:${CMAKE_BINARY_DIR}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib/vicon_sdk:${CMAKE_BINARY_DIR}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/vicon_sdk:${CMAKE_BINARY_DIR}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(argos3)
add_subdirectory(vicon_examples)

file(GLOB VICON_GLOB "${VICON_PATH}/*.so*" "${VICON_PATH}/*.dylib")
foreach(ITEM ${VICON_GLOB})
   install(FILES "${ITEM}" DESTINATION lib/vicon_sdk)
endforeach()

if(APPLE)
  foreach(ITEM ${VICON_GLOB})
    install(FILES "${ITEM}" DESTINATION lib/argos3)
  endforeach()
endif(APPLE)

install(FILES ${VICON_PATH}/DataStreamClient.h DESTINATION include/vicon_sdk)

set(CMAKE_CXX_EXTENSIONS OFF)


