/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  Author(s):  Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnQt.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciQt.h>

#include <cisst_ros_bridge/mtsROSBridge.h>
#include <isi_ros/isi_ros.h>

#include <QApplication>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsIntuitiveDaVinci", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    cisst_ral::ral ral(argc, argv, "intuitive_da_vinci");
    auto rosNode = ral.node();

    // parse options
    cmnCommandLineOptions options;

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    double rosPeriod = 20.0 * cmn_ms; // isi api defined
    double tfPeriod = 20.0 * cmn_ms;
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the da Vinci",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the da Vinci",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);

    std::list<std::string> managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    if (!options.Parse(argc, argv, std::cerr)) {
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
        cmnQt::QApplicationExitsOnCtrlC();
        if (options.IsSet("dark-mode")) {
            cmnQt::SetDarkMode();
        }
        daVinciQt = new mtsIntuitiveDaVinciQt();
        daVinciQt->Configure(daVinci);
        daVinciQt->Connect();
    }

    // ros wrapper
    isi_ros * isiROS = new isi_ros(daVinci->GetName(), rosNode, rosPeriod, tfPeriod);
    componentManager->AddComponent(isiROS);
    isiROS->Connect();

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
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

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    cisst_ral::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    if (daVinciQt) {
        delete daVinciQt;
    }
    delete daVinci;

    return 0;
}
