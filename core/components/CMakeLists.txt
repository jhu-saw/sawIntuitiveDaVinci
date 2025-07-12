#
# (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawIntuitiveDaVinci VERSION 2.0.0)

set (
  REQUIRED_CISST_LIBRARIES cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to make sure CISST_USE_SI_UNITS to ON")
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  # To find isi_api-config.cmake
  find_package (isi_api)

  if (isi_api_FOUND)

    # catkin/ROS paths
    cisst_is_catkin_build (sawIntuitiveDaVinci_IS_CATKIN_BUILT)
    if (sawIntuitiveDaVinci_IS_CATKIN_BUILT)
      set (sawIntuitiveDaVinci_CONFIG_FILE_DIR
           "${CATKIN_DEVEL_PREFIX}/share/sawIntuitiveDaVinci/cmake")
      set (LIBRARY_OUTPUT_PATH "${sawIntuitiveDaVinci_BINARY_DIR}/lib")
    else ()
      set (sawIntuitiveDaVinci_CONFIG_FILE_DIR "${sawIntuitiveDaVinci_BINARY_DIR}")
    endif ()

    set (sawIntuitiveDaVinci_INCLUDE_DIR
      "${sawIntuitiveDaVinci_SOURCE_DIR}/include"
      "${sawIntuitiveDaVinci_BINARY_DIR}/include")
    set (sawIntuitiveDaVinci_HEADER_DIR "${sawIntuitiveDaVinci_SOURCE_DIR}/include/sawIntuitiveDaVinci")
    set (sawIntuitiveDaVinci_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawIntuitiveDaVinci_LIBRARIES sawIntuitiveDaVinci "${isi_api_LIBRARIES}")

    include_directories (BEFORE
      "${sawIntuitiveDaVinci_INCLUDE_DIR}"
      "${isi_api_INCLUDE_DIR}")

    # add Qt code
    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawIntuitiveDaVinci_LIBRARIES ${sawIntuitiveDaVinci_LIBRARIES} sawIntuitiveDaVinciQt)
    endif (CISST_HAS_QT)

    # add all config files for this component
    cisst_add_config_files (sawIntuitiveDaVinci)

    add_library (
      sawIntuitiveDaVinci ${IS_SHARED}
      ${sawIntuitiveDaVinci_HEADER_DIR}/sawIntuitiveDaVinciExport.h
      code/mtsIntuitiveDaVinci.cpp
      ${sawIntuitiveDaVinci_HEADER_DIR}/mtsIntuitiveDaVinci.h
      ${sawIntuitiveDaVinci_BINARY_DIR}/include/sawIntuitiveDaVinci/sawIntuitiveDaVinciRevision.h)

    set_target_properties (sawIntuitiveDaVinci PROPERTIES
      VERSION ${sawIntuitiveDaVinci_VERSION}
      FOLDER "sawIntuitiveDaVinci")

    cisst_target_link_libraries (sawIntuitiveDaVinci ${REQUIRED_CISST_LIBRARIES})
    # We can't link since the ISI binaries were not compiled using -fPIC
    # target_link_libraries (sawIntuitiveDaVinci ${isi_api_LIBRARIES})

    # Install target for headers and library
    install (
      DIRECTORY
      ${sawIntuitiveDaVinci_SOURCE_DIR}/include/sawIntuitiveDaVinci
      ${sawIntuitiveDaVinci_BINARY_DIR}/include/sawIntuitiveDaVinci
      DESTINATION include
      COMPONENT sawIntuitiveDaVinci-dev)

    install (
      TARGETS sawIntuitiveDaVinci
      COMPONENT sawIntuitiveDaVinci
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (
      FILES "${sawIntuitiveDaVinci_BINARY_DIR}/sawIntuitiveDaVinciConfig.cmake"
      DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  else (isi_api_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires Intuitive Surgical Inc isi_api.  Please set isi_api_DIR using CMake, e.g. ccmake ${CMAKE_CURRENT_BINARY_DIR}")
  endif (isi_api_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
