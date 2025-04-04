# It defines the following variables
#  OCTOVIS_INCLUDE_DIRS  - include directories for OctoMap viewer
#  OCTOVIS_LIBRARY_DIRS  - library directories for OctoMap viewer
#  OCTOVIS_LIBRARIES     - libraries to link against
#  OCTOVIS_MAJOR_VERSION - major version
#  OCTOVIS_MINOR_VERSION - minor version
#  OCTOVIS_PATCH_VERSION - patch version
#  OCTOVIS_VERSION       - major.minor.patch version


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was octovis-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(OCTOVIS_MAJOR_VERSION "1")
set(OCTOVIS_MINOR_VERSION "10")
set(OCTOVIS_PATCH_VERSION "0")
set(OCTOVIS_VERSION "1.10.0")

set_and_check(OCTOVIS_INCLUDE_DIRS "/home/research/hc_ws/src/OctoMap/octomap/octovis/include" "/home/research/hc_ws/src/OctoMap/octomap/octovis/src/extern/QGLViewer")
set_and_check(OCTOVIS_LIBRARY_DIRS "/home/research/hc_ws/src/OctoMap/octomap/lib" "")

# Set library names as absolute paths:
set(OCTOVIS_LIBRARIES
  "/home/research/hc_ws/src/OctoMap/octomap/octovis/src/extern/QGLViewer/libQGLViewer.so"
  "Qt5::Core;Qt5::Gui;Qt5::OpenGL;Qt5::Widgets;Qt5::Xml;/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libGLU.so"
  "/home/research/hc_ws/src/OctoMap/octomap/lib/liboctovis.so"
)


