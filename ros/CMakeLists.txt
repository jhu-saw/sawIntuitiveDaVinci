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

cmake_minimum_required (VERSION 2.8.3)

project (isi_ros)

find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_bridge
              geometry_msgs
              roscpp
              std_msgs
              roslib
              )

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

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS}
                       ${isi_ros_SOURCE_DIR}/include)

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  LIBRARIES isi_ros
                  CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp std_msgs)

  find_package (sawIntuitiveDaVinci 1.2.0)

  if (sawIntuitiveDaVinci_FOUND)

    # sawIntuitiveDaVinci configuration
    include_directories (${sawIntuitiveDaVinci_INCLUDE_DIR})
    link_directories (${sawIntuitiveDaVinci_LIBRARY_DIR})

    add_library (isi_ros
                 include/isi_ros/isi_ros.h
                 src/isi_ros.cpp)

    target_link_libraries (isi_ros
                           ${sawIntuitiveDaVinci_LIBRARIES}
                           )
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (isi_ros ${REQUIRED_CISST_LIBRARIES})

    # name the main executable and specifies with source files to use
    add_executable (isi_console
                    src/isi_console.cpp
                    )
    set_property (TARGET isi_console PROPERTY FOLDER "sawIntuitiveDaVinci")

    # link against non cisst libraries and saw components
    target_link_libraries (isi_console
                           ${sawIntuitiveDaVinci_LIBRARIES}
                           isi_ros
			   ${catkin_LIBRARIES})
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (isi_console ${REQUIRED_CISST_LIBRARIES})

  endif (sawIntuitiveDaVinci_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
