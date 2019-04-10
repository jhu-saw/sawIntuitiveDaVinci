#
# (C) Copyright 2009-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# list of cisst libraries needed
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveDaVinciExamples_IS_CATKIN_BUILT)
  if (sawIntuitiveDaVinciExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawIntuitiveDaVinci has been compiled within cisst, we should find it automatically
  find_package (sawIntuitiveDaVinci 1.2.0)

  if (sawIntuitiveDaVinci_FOUND)

    # sawIntuitiveDaVinci configuration
    include_directories (${sawIntuitiveDaVinci_INCLUDE_DIR})
    link_directories (${sawIntuitiveDaVinci_LIBRARY_DIR})

    # name the main executable and specifies with source files to use
    add_executable (sawIntuitiveDaVinciConsoleQt
                    console.cpp
                    )
    set_property (TARGET sawIntuitiveDaVinciConsoleQt PROPERTY FOLDER "sawIntuitiveDaVinci")

    # link against non cisst libraries and saw components
    target_link_libraries (sawIntuitiveDaVinciConsoleQt
                           ${sawIntuitiveDaVinci_LIBRARIES}
                           )
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawIntuitiveDaVinciConsoleQt ${REQUIRED_CISST_LIBRARIES})

  endif (sawIntuitiveDaVinci_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
