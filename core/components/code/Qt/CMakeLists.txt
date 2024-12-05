#
# (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstVectorQt
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  set (CMAKE_AUTOMOC ON)

  add_library (sawIntuitiveDaVinciQt
    ${sawIntuitiveDaVinci_HEADER_DIR}/mtsIntuitiveDaVinciQt.h
    ${sawIntuitiveDaVinci_HEADER_DIR}/mtsIntuitiveDaVinciArmQtWidget.h
    mtsIntuitiveDaVinciQt.cpp
    mtsIntuitiveDaVinciArmQtWidget.cpp
    ${SAW_INTUITIVE_RESEARCH_KIT_QT_WRAP_CPP})

  set_target_properties (
    sawIntuitiveDaVinciQt PROPERTIES
    VERSION ${sawIntuitiveDaVinci_VERSION}
    FOLDER "sawIntuitiveDaVinci")
  cisst_target_link_libraries (sawIntuitiveDaVinciQt ${REQUIRED_CISST_LIBRARIES})

  install (
    TARGETS sawIntuitiveDaVinciQt
    COMPONENT sawIntuitiveDaVinci-Qt
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
