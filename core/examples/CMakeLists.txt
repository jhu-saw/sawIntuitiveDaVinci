#
# (C) Copyright 2009-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawIntuitiveDaVinciExamples VERSION 2.0.0)

# list of cisst libraries needed
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # sawIntuitiveDaVinci has been compiled within cisst, we should find it automatically
  find_package (sawIntuitiveDaVinci 2.0.0)

  if (sawIntuitiveDaVinci_FOUND)

    # sawIntuitiveDaVinci configuration
    include_directories (${sawIntuitiveDaVinci_INCLUDE_DIR})
    link_directories (${sawIntuitiveDaVinci_LIBRARY_DIR})

    # name the main executable and specifies with source files to use
    add_executable (
      sawIntuitiveDaVinciConsoleQt
      console.cpp)
    set_target_properties (
      sawIntuitiveDaVinciConsoleQt PROPERTIES
      COMPONENT sawIntuitiveDaVinci-Examples
      FOLDER "sawIntuitiveDaVinci")

    # until lib isi_api gets compile with -fPIC we might need this
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")

    # link against non cisst libraries and saw components
    target_link_libraries (
      sawIntuitiveDaVinciConsoleQt
      ${sawIntuitiveDaVinci_LIBRARIES})
    
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawIntuitiveDaVinciConsoleQt ${REQUIRED_CISST_LIBRARIES})

    install (
      TARGETS sawIntuitiveDaVinciConsoleQt
      COMPONENT sawIntuitiveDaVinci-Examples
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

  endif (sawIntuitiveDaVinci_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
