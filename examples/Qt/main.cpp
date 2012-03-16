/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: main.cpp 3238 2011-12-08 16:14:30Z adeguet1 $

  Author(s):  Praneeth Sadda, Anton Deguet
  Created on: 2011-11-11

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <cisstMultiTask/mtsQtWidgetComponent.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>

#include <QApplication>
#include <QMainWindow>

#include <cisstMultiTask/mtsTaskManager.h>

int main(int argc, char** argv)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mts", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    mtsComponentManager * manager = mtsComponentManager::GetInstance();
    mtsIntuitiveDaVinci * daVinci = new mtsIntuitiveDaVinci("daVinci", 50.0 * cmn_ms);
    daVinci->Configure("");
    manager->AddComponent(daVinci);

    QApplication app(argc, argv);

    mtsQtWidgetComponent * componentWidget = new mtsQtWidgetComponent("QtWidgetComponent");
    componentWidget->CreateWidgetsForComponent(*daVinci);
    QMainWindow win;
    win.setCentralWidget(componentWidget);
    win.show();

    manager->CreateAll();
    manager->WaitForStateAll(mtsComponentState::READY, 5.0 * cmn_s);
    manager->StartAll();
    manager->WaitForStateAll(mtsComponentState::ACTIVE, 5.0 * cmn_s);

    app.exec();

    return 0;
}
