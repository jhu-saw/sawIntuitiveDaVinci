/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  Author(s):  Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>

// cisst/saw
#include <cisstCommon/cmnCommandLineOptions.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciQt.h>

#include <QApplication>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsIntuitiveDaVinci", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string managerConfig;
    options.AddOptionOneValue("c", "component-manager",
                              "JSON file to configure component manager",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    QApplication application(argc, argv);

    // daVinci wrapper
    mtsIntuitiveDaVinci * daVinci = new mtsIntuitiveDaVinci("daVinci", 50 /* Hz */);
    daVinci->Configure();
    componentManager->AddComponent(daVinci);

    mtsIntuitiveDaVinciQt * daVinciQt = new mtsIntuitiveDaVinciQt();
    daVinciQt->Configure(daVinci);
    daVinciQt->Connect();

    // custom user component
    if (!managerConfig.empty()) {
        if (!cmnPath::Exists(managerConfig)) {
            CMN_LOG_INIT_ERROR << "File " << managerConfig
                               << " not found!" << std::endl;
        } else {
            if (!componentManager->ConfigureJSON(managerConfig)) {
                CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager" << std::endl;
                return -1;
            }
        }
    }
    
    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    application.exec();

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    delete daVinciQt;
    delete daVinci;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
