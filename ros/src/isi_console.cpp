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

    // parse options
    cmnCommandLineOptions options;
    std::string rosNamespace = "isi";
    double rosPeriod = 20.0 * cmn_ms; // isi api defined
    double tfPeriod = 20.0 * cmn_ms;

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the da Vinci",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the da Vinci",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);

    options.AddOptionOneValue("n", "ros-namespace",
                              "ROS namespace to prefix all topics, must have start and end \"/\" (default /isi)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosNamespace);

    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
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

    // ros wrapper
    std::string bridgeName = "sawIntuitiveDaVinci" + rosNamespace;
    std::replace(bridgeName.begin(), bridgeName.end(), '/', '_');
    mtsROSBridge * rosBridge = new mtsROSBridge(bridgeName, rosPeriod, true);
    mtsROSBridge * tfBridge = new mtsROSBridge(bridgeName + "_tf2", tfPeriod, true);

    isi_ros * isiROS = new isi_ros(rosBridge, tfBridge, rosNamespace, daVinci);

    componentManager->AddComponent(rosBridge);
    componentManager->AddComponent(tfBridge);
    isiROS->Connect();

    // custom user component
    const managerConfigType::iterator end = managerConfig.end();
    for (managerConfigType::iterator iter = managerConfig.begin();
         iter != end;
         ++iter) {
        if (!iter->empty()) {
            if (!cmnPath::Exists(*iter)) {
                CMN_LOG_INIT_ERROR << "File " << *iter
                                   << " not found!" << std::endl;
            } else {
                if (!componentManager->ConfigureJSON(*iter)) {
                    CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager" << std::endl;
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

    if (daVinciQt) {
        delete daVinciQt;
    }
    delete daVinci;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
