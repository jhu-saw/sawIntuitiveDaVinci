#
#
# Author(s):  Anton Deguet
# Created on: 2011-01-20
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
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

    set (sawIntuitiveDaVinci_INCLUDE_DIR "${sawIntuitiveDaVinci_SOURCE_DIR}/include")
    
    if ("${LIBRARY_OUTPUT_PATH}" STREQUAL "")
      set (sawIntuitiveDaVinci_LIBRARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    else ()
      set (sawIntuitiveDaVinci_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    endif ()
    set (sawIntuitiveDaVinci_LIBRARIES sawIntuitiveDaVinci "${isi_api_LIBRARIES}")

    include_directories ("${sawIntuitiveDaVinci_INCLUDE_DIR}" "${isi_api_INCLUDE_DIR}")
    set (sawIntuitiveDaVinci_HEADER_DIR "${sawIntuitiveDaVinci_INCLUDE_DIR}/sawIntuitiveDaVinci")
    add_library (sawIntuitiveDaVinci ${IS_SHARED}
                 ${sawIntuitiveDaVinci_HEADER_DIR}/sawIntuitiveDaVinciExport.h
                 mtsIntuitiveDaVinci.cpp
                 ${sawIntuitiveDaVinci_HEADER_DIR}/mtsIntuitiveDaVinci.h)
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

    # Install target for headers and library
    install (DIRECTORY
             ${sawIntuitiveDaVinci_INCLUDE_DIR}/sawIntuitiveDaVinci
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
