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

#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciQt.h>

// cisst/saw
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstParameterTypes/prmEventButtonQtWidget.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciArmQtWidget.h>
#include <QTabWidget>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveDaVinciQt);

mtsIntuitiveDaVinciQt::mtsIntuitiveDaVinciQt(void)
{
    Arms["MTML1"] = 0;
    Arms["MTMR1"] = 0;
    Arms["PSM1"] = 0;
    Arms["PSM2"] = 0;
    Arms["PSM3"] = 0;
    Arms["ECM1"] = 0;
}

mtsIntuitiveDaVinciQt::~mtsIntuitiveDaVinciQt()
{
    ArmsType::iterator iter;
    const ArmsType::iterator end = Arms.end();

    for (iter = Arms.begin();
         iter != end;
         ++iter) {
        if (iter->second) {
            delete iter->second;
        }
    }
}

void mtsIntuitiveDaVinciQt::Configure(mtsIntuitiveDaVinci * daVinci)
{
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    TabWidget = new QTabWidget;

    ArmsType::iterator iter;
    const ArmsType::iterator end = Arms.end();

    for (iter = Arms.begin();
         iter != end;
         ++iter) {
        iter->second = new mtsIntuitiveDaVinciArmQtWidget(iter->first + "-GUI");
        iter->second->Configure();
        componentManager->AddComponent(iter->second);
        Connections.push_back(new ConnectionType(iter->second->GetName(), "Manipulator",
                                                 daVinci->GetName(), iter->first));
        TabWidget->addTab(iter->second, iter->first.c_str());
    }

    // Event (Buttons only)
    std::vector<std::string> buttons;
    buttons.push_back("MTML1Select");
    buttons.push_back("MTML1Clutch");
    buttons.push_back("MTMR1Select");
    buttons.push_back("MTMR1Clutch");
    buttons.push_back("Standby");
    buttons.push_back("Ready");
    buttons.push_back("Clutch");
    buttons.push_back("Camera");
    buttons.push_back("FollowMode");
    buttons.push_back("MastersAsMice");

    prmEventButtonQtWidgetComponent * buttonsGUI = new prmEventButtonQtWidgetComponent("Buttons");
    componentManager->AddComponent(buttonsGUI);
    buttonsGUI->SetNumberOfColumns(4);
    for (size_t index = 0;
         index < buttons.size();
         ++index) {
        buttonsGUI->AddEventButton(buttons[index]);
        Connections.push_back(new ConnectionType(buttonsGUI->GetName(), buttons[index],
                                                 daVinci->GetName(), buttons[index]));
    }
    TabWidget->addTab(buttonsGUI, "Buttons");

    // show all widgets
    TabWidget->show();
}

void mtsIntuitiveDaVinciQt::Connect(void)
{
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    const ConnectionsType::const_iterator end = Connections.end();
    ConnectionsType::const_iterator connectIter;
    for (connectIter = Connections.begin();
         connectIter != end;
         ++connectIter) {
        ConnectionType * connection = *connectIter;
        componentManager->Connect(connection->ClientComponentName,
                                  connection->ClientInterfaceName,
                                  connection->ServerComponentName,
                                  connection->ServerInterfaceName);
    }
}
