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

# create a list of libraries needed for this project
set (REQUIRED_CISST_LIBRARIES cisstCommon cisstVector cisstOSAbstraction cisstNumerical
                              cisstMultiTask cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawIntuitiveDaVinci has been compiled within cisst, we should find it automatically
  find_package (sawIntuitiveDaVinci REQUIRED HINTS "${CISST_BINARY_DIR}/saw/components")

  if (sawIntuitiveDaVinci_FOUND)

    # sawDaVinciISIAP configuration
    include_directories (${sawIntuitiveDaVinci_INCLUDE_DIR})
    link_directories (${sawIntuitiveDaVinci_LIBRARY_DIR})

    add_executable (sawIntuitiveDaVinciExample
                    main.cpp)

    cisst_target_link_libraries (sawIntuitiveDaVinciExample ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries (sawIntuitiveDaVinciExample ${sawIntuitiveDaVinci_LIBRARIES})

  endif (sawIntuitiveDaVinci_FOUND)

endif (cisst_FOUND)
