#
# $Id$
#
# Author(s):  Anton Deguet
# Created on: 2011-01-20
#
# (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
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

  add_library (sawIntuitiveDaVinci ${IS_SHARED}
               ../sawIntuitiveDaVinciExport.h
               mtsIntuitiveDaVinci.cpp
               ../mtsIntuitiveDaVinci.h)

  target_link_libraries (sawIntuitiveDaVinci
                         ${isi_api_LIBRARIES})
  cisst_target_link_libraries (sawIntuitiveDaVinci ${REQUIRED_CISST_LIBRARIES})

  set (sawIntuitiveDaVinci_INCLUDE_DIR "${sawComponents_SOURCE_DIR}")
  set (sawIntuitiveDaVinci_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawIntuitiveDaVinci_LIBRARIES sawIntuitiveDaVinci "${isi_api_LIBRARIES}")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawIntuitiveDaVinciConfig.cmake.in"
                  "${sawComponents_BINARY_DIR}/sawIntuitiveDaVinciConfig.cmake"
                  @ONLY@)

endif (cisst_FOUND)
