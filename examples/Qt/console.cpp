/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Author(s):  Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsQtApplication.h>
#include <cisstParameterTypes/prmQtWidgetEventButtonsComponent.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciArmQtWidget.h>


#include <QTabWidget>

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
    std::string gcmip = "-1";

    options.AddOptionOneValue("g", "gcmip",
                              "global component manager IP address",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &gcmip);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    std::string processName = "daVinci";
    mtsManagerLocal * componentManager = 0;
    if (gcmip != "-1") {
        try {
            componentManager = mtsManagerLocal::GetInstance(gcmip, processName);
        } catch(...) {
            std::cerr << "Failed to get GCM instance." << std::endl;
            return -1;
        }
    } else {
        componentManager = mtsManagerLocal::GetInstance();
    }

    // create a Qt application and tab to hold all widgets
    mtsQtApplication * qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);

    // daVinci wrapper
    mtsIntuitiveDaVinci * daVinci = new mtsIntuitiveDaVinci("daVinci", 50 /* Hz */);
    daVinci->Configure();
    componentManager->AddComponent(daVinci);

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // Arms
    typedef std::map<std::string, mtsIntuitiveDaVinciArmQtWidget *> ArmsType;
    ArmsType arms;
    arms["MTML1"] = 0;
    arms["MTMR1"] = 0;
    arms["PSM1"] = 0;
    arms["PSM2"] = 0;
    arms["PSM3"] = 0;
    arms["ECM1"] = 0;

    ArmsType::iterator iter;
    ArmsType::iterator end = arms.end();

    for (iter = arms.begin();
         iter != end;
         ++iter) {
        iter->second = new mtsIntuitiveDaVinciArmQtWidget(iter->first + "-GUI");
        iter->second->Configure();
        componentManager->AddComponent(iter->second);
        componentManager->Connect(iter->second->GetName(), "Manipulator",
                                  daVinci->GetName(), iter->first);
        tabWidget->addTab(iter->second, iter->first.c_str());
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

    prmQtWidgetEventButtonsComponent * buttonsGUI = new prmQtWidgetEventButtonsComponent("Buttons");
    componentManager->AddComponent(buttonsGUI);
    buttonsGUI->SetNumberOfColumns(4);
    for (size_t index = 0;
         index < buttons.size();
         ++index) {
        buttonsGUI->AddEventButton(buttons[index]);
        componentManager->Connect(buttonsGUI->GetName(), buttons[index],
                                  daVinci->GetName(), buttons[index]);
    }
    tabWidget->addTab(buttonsGUI, "Buttons");

    tabWidget->show();

    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // QtApplication will run in main thread and return control
    // when exited.

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    for (iter = arms.begin();
         iter != end;
         ++iter) {
        delete iter->second;
    }
    delete daVinci;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
