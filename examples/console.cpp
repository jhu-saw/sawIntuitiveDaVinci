/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  Author(s):  Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnGetChar.h>
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
    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    const bool hasQt = !options.IsSet("text-only");

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // daVinci wrapper
    mtsIntuitiveDaVinci * daVinci = new mtsIntuitiveDaVinci("daVinci", 50 /* Hz */);
    daVinci->Configure();
    componentManager->AddComponent(daVinci);

    // add all Qt widgets if needed
    QApplication * application = 0;
    mtsIntuitiveDaVinciQt * daVinciQt = 0;
    if (hasQt) {
        application = new QApplication(argc, argv);
        daVinciQt = new mtsIntuitiveDaVinciQt();
        daVinciQt->Configure(daVinci);
        daVinciQt->Connect();
    }

    // custom user component
    const managerConfigType::iterator endConfig = managerConfig.end();
    for (managerConfigType::iterator iterConfig = managerConfig.begin();
         iterConfig != endConfig;
         ++iterConfig) {
        if (!iterConfig->empty()) {
            if (!cmnPath::Exists(*iterConfig)) {
                CMN_LOG_INIT_ERROR << "File " << *iterConfig
                                   << " not found!" << std::endl;
            } else {
                if (!componentManager->ConfigureJSON(*iterConfig)) {
                    CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager for "
                                       << *iterConfig << std::endl;
                    return -1;
                }
            }
        }
    }

    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    if (hasQt) {
        application->exec();
    } else {
        do {
            std::cout << "Press 'q' to quit" << std::endl;
        } while (cmnGetChar() != 'q');
    }

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    delete daVinciQt;
    delete daVinci;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
