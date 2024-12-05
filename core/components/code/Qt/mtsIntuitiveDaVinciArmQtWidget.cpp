/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>
#include <QMessageBox>
#include <QScrollBar>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciArmQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveDaVinciArmQtWidget, mtsComponent, std::string);

mtsIntuitiveDaVinciArmQtWidget::mtsIntuitiveDaVinciArmQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    QSJWidget = new prmStateJointQtWidget();
    QSysWidget = new mtsSystemQtWidget(componentName);

    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Manipulator");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", Arm.measured_cp);
        interfaceRequired->AddFunction("configuration_js", Arm.configuration_js);
        interfaceRequired->AddFunction("measured_js", Arm.measured_js);
        QSysWidget->SetInterfaceRequired(interfaceRequired);
        interfaceRequired->AddFunction("period_statistics", Arm.period_statistics);
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsIntuitiveDaVinciArmQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveDaVinciArmQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveDaVinciManipulatorQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsIntuitiveDaVinciArmQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveDaVinciManipulatorQtWidget::Cleanup" << std::endl;
}

void mtsIntuitiveDaVinciArmQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsIntuitiveDaVinciArmQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsIntuitiveDaVinciArmQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = Arm.measured_cp(m_measured_cp);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Manipulator.measured_cp failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QPCGWidget->SetValue(m_measured_cp);

    executionResult = Arm.measured_js(m_measured_js);
    if (executionResult) {
        if ((m_configuration_js.Name().size() != m_measured_js.Name().size())
            && (Arm.configuration_js.IsValid())) {
            Arm.configuration_js(m_configuration_js);
            QSJWidget->SetConfiguration(m_configuration_js);
        }
        QSJWidget->SetValue(m_measured_js);
    }
    QSJWidget->SetValue(m_measured_js);

    Arm.period_statistics(IntervalStatistics);
    QSysWidget->SetValue(IntervalStatistics);
}

void mtsIntuitiveDaVinciArmQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    // Side by side for 3D position and timing
    QHBoxLayout * topLayout = new QHBoxLayout;
    mainLayout->addLayout(topLayout);

    // 3D position
    QPCGWidget = new prmPositionCartesianGetQtWidget();
    QPCGWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    topLayout->addWidget(QPCGWidget);

    // System
    QSysWidget->setupUi();
    topLayout->addWidget(QSysWidget);

    // Joint state
    QSJWidget->setupUi();
    QSJWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    mainLayout->addWidget(QSJWidget);

    setLayout(mainLayout);
    setWindowTitle("Manipulator");
    resize(sizeHint());
}
