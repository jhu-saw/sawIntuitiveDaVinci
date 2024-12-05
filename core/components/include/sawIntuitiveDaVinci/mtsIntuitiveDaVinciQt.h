/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveDaVinciQt_h
#define _mtsIntuitiveDaVinciQt_h

#include <cisstCommon/cmnGenericObject.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciArmQtWidget.h>
class QTabWidget;
class QWidget;

class mtsIntuitiveDaVinciQt: public cmnGenericObject
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveDaVinciQt(void);
    ~mtsIntuitiveDaVinciQt();
    void Configure(mtsIntuitiveDaVinci * daVinci);
    void Connect(void);

protected:
    typedef std::map<std::string, mtsIntuitiveDaVinciArmQtWidget *> ArmsType;
    ArmsType Arms;

    class ConnectionType {
    public:
        inline ConnectionType(const std::string & clientComponentName,
                              const std::string & clientInterfaceName,
                              const std::string & serverComponentName,
                              const std::string & serverInterfaceName):
            ClientComponentName(clientComponentName),
            ClientInterfaceName(clientInterfaceName),
            ServerComponentName(serverComponentName),
            ServerInterfaceName(serverInterfaceName)
        {}

        std::string ClientComponentName;
        std::string ClientInterfaceName;
        std::string ServerComponentName;
        std::string ServerInterfaceName;
    };

    typedef std::list<ConnectionType *> ConnectionsType;
    ConnectionsType Connections;

    QTabWidget * TabWidget;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveDaVinciQt);

#endif // _mtsIntuitiveDaVinciQt_h
