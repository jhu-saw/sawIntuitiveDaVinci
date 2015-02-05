#
# Author(s):  Anton Deguet
# Created on: 2011-01-20
#
# (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---


set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # To find isi_api-config.cmake
  find_package (isi_api REQUIRED)

  if (isi_api_FOUND)

    # where to save the libraries
    if (NOT DEFINED LIBRARY_OUTPUT_PATH)
      set (LIBRARY_OUTPUT_PATH "${sawIntuitiveDaVinci_BINARY_DIR}/lib")
    endif ()

    set (sawIntuitiveDaVinci_INCLUDE_DIR "${sawIntuitiveDaVinci_SOURCE_DIR}/include"
                                         "${sawIntuitiveDaVinci_BINARY_DIR}/include")
    set (sawIntuitiveDaVinci_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawIntuitiveDaVinci_LIBRARIES sawIntuitiveDaVinci "${isi_api_LIBRARIES}")
    set (sawIntuitiveDaVinci_CONFIG_FILE_DIR "${sawIntuitiveDaVinci_BINARY_DIR}")

    # Set the version number
    set (sawIntuitiveDaVinci_VERSION_MAJOR "1")
    set (sawIntuitiveDaVinci_VERSION_MINOR "0")
    set (sawIntuitiveDaVinci_VERSION_PATCH "1")
    set (sawIntuitiveDaVinci_VERSION "${sawIntuitiveDaVinci_VERSION_MAJOR}.${sawIntuitiveDaVinci_VERSION_MINOR}.${sawIntuitiveDaVinci_VERSION_PATCH}")

    # Generate sawIntuitiveDaVinciRevision.h
    configure_file ("${sawIntuitiveDaVinci_SOURCE_DIR}/code/sawIntuitiveDaVinciRevision.h.in"
                    "${sawIntuitiveDaVinci_BINARY_DIR}/include/sawIntuitiveDaVinci/sawIntuitiveDaVinciRevision.h")

    include_directories (${sawIntuitiveDaVinci_INCLUDE_DIR} "${isi_api_INCLUDE_DIR}")
    set (sawIntuitiveDaVinci_HEADER_DIR "${sawIntuitiveDaVinci_SOURCE_DIR}/include/sawIntuitiveDaVinci")
    add_library (sawIntuitiveDaVinci ${IS_SHARED}
                 ${sawIntuitiveDaVinci_HEADER_DIR}/sawIntuitiveDaVinciExport.h
                 mtsIntuitiveDaVinci.cpp
                 ${sawIntuitiveDaVinci_HEADER_DIR}/mtsIntuitiveDaVinci.h
                 ${sawIntuitiveDaVinci_BINARY_DIR}/include/sawIntuitiveDaVinci/sawIntuitiveDaVinciRevision.h)

    set_property (TARGET sawIntuitiveDaVinci PROPERTY FOLDER "sawIntuitiveDaVinci")
    target_link_libraries (sawIntuitiveDaVinci
                           ${isi_api_LIBRARIES})
    cisst_target_link_libraries (sawIntuitiveDaVinci ${REQUIRED_CISST_LIBRARIES})

    # add Qt code
    add_subdirectory (Qt)
	set (sawIntuitiveDaVinci_LIBRARY_DIR ${sawIntuitiveDaVinci_LIBRARY_DIR} ${sawIntuitiveDaVinciQt_LIBRARY_DIR})
    set (sawIntuitiveDaVinci_LIBRARIES ${sawIntuitiveDaVinci_LIBRARIES} ${sawIntuitiveDaVinciQt_LIBRARIES})

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawIntuitiveDaVinciConfig.cmake.in"
                    "${sawIntuitiveDaVinci_BINARY_DIR}/sawIntuitiveDaVinciConfig.cmake"
                    @ONLY@)

    cisst_add_config_version ("${sawIntuitiveDaVinci_CONFIG_FILE_DIR}/sawIntuitiveDaVinciConfigVersion.cmake"
                              VERSION ${sawIntuitiveDaVinci_VERSION}
                              DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                              COMPONENT sawIntuitiveDaVinci)

    # Install target for headers and library
    install (DIRECTORY
             ${sawIntuitiveDaVinci_SOURCE_DIR}/include/sawIntuitiveDaVinci
             ${sawIntuitiveDaVinci_BINARY_DIR}/include/sawIntuitiveDaVinci
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawIntuitiveDaVinci
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawIntuitiveDaVinci_BINARY_DIR}/sawIntuitiveDaVinciConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  else (isi_api_FOUND)
    message (SEND_ERROR "code in ${CMAKE_CURRENT_SOURCE_DIR} requires Intuitive Surgical Inc isi_api.")
  endif (isi_api_FOUND)

endif (cisst_FOUND)
