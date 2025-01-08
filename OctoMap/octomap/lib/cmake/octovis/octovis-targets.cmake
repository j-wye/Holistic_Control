# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.6)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.20)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget octovis octovis-static octovis-shared)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target octovis
add_executable(octovis IMPORTED)

# Create imported target octovis-static
add_library(octovis-static STATIC IMPORTED)

set_target_properties(octovis-static PROPERTIES
  INTERFACE_LINK_LIBRARIES "/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libGLU.so;/home/research/hc_ws/src/OctoMap/octomap/lib/liboctomap.so;/home/research/hc_ws/src/OctoMap/octomap/lib/liboctomath.so;/home/research/hc_ws/src/OctoMap/octomap/octovis/src/extern/QGLViewer/libQGLViewer.so"
)

# Create imported target octovis-shared
add_library(octovis-shared SHARED IMPORTED)

set_target_properties(octovis-shared PROPERTIES
  INTERFACE_LINK_LIBRARIES "/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libGLU.so;/home/research/hc_ws/src/OctoMap/octomap/lib/liboctomap.so;/home/research/hc_ws/src/OctoMap/octomap/lib/liboctomath.so;/home/research/hc_ws/src/OctoMap/octomap/octovis/src/extern/QGLViewer/libQGLViewer.so"
)

# Import target "octovis" for configuration "Release"
set_property(TARGET octovis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(octovis PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/research/hc_ws/src/OctoMap/octomap/bin/octovis"
  )

# Import target "octovis-static" for configuration "Release"
set_property(TARGET octovis-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(octovis-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/home/research/hc_ws/src/OctoMap/octomap/lib/liboctovis.a"
  )

# Import target "octovis-shared" for configuration "Release"
set_property(TARGET octovis-shared APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(octovis-shared PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/research/hc_ws/src/OctoMap/octomap/lib/liboctovis.so.1.10.0"
  IMPORTED_SONAME_RELEASE "liboctovis.so.1.10"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)