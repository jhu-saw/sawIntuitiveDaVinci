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
project (intuitive_da_vinci VERSION 1.1.0)

# first test for ROS1
find_package (catkin QUIET
  COMPONENTS
  cisst_ros_crtk
  roscpp
  roslib)

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (REQUIRED_AMENT_PACKAGES
      rclcpp std_msgs std_srvs geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs cisst_msgs crtk_msgs cisst_ros_bridge cisst_ros_crtk)
    foreach (_pkg ${REQUIRED_AMENT_PACKAGES})
      find_package (${_pkg} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
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

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawIntuitiveDaVinci has been compiled within cisst, we should find it automatically
  find_package (sawIntuitiveDaVinci 2.0.0)

  if (sawIntuitiveDaVinci_FOUND)

    if (ROS_VERSION EQUAL 1)
      file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
      catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
        CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)
      include_directories (${catkin_INCLUDE_DIRS})
    endif ()

    # sawIntuitiveDaVinci configuration
    include_directories (${sawIntuitiveDaVinci_INCLUDE_DIR} "${intuitive_da_vinci_SOURCE_DIR}/include")
    link_directories (${sawIntuitiveDaVinci_LIBRARY_DIR})

    add_executable (intuitive_da_vinci
      include/isi_ros/isi_ros.h
      src/isi_ros.cpp
      src/isi_console.cpp)

    # until lib isi_api gets compile with -fPIC we might need this
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")

    target_link_libraries (intuitive_da_vinci
      ${sawIntuitiveDaVinci_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (intuitive_da_vinci ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS intuitive_da_vinci
      LIBRARY DESTINATION lib/${PROJECT_NAME}
      RUNTIME DESTINATION lib/${PROJECT_NAME}
      LIBRARY DESTINATION lib
      INCLUDES DESTINATION include)

    if (ROS_VERSION EQUAL 2)
      ament_target_dependencies (
        intuitive_da_vinci
        ${REQUIRED_AMENT_PACKAGES}
        )
      ament_package ()
    endif ()

  else (sawIntuitiveDaVinci_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawIntuitiveDaVinci")
  endif (sawIntuitiveDaVinci_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
