/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

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

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinciArmQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveDaVinciArmQtWidget, mtsComponent, std::string);

mtsIntuitiveDaVinciArmQtWidget::mtsIntuitiveDaVinciArmQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Manipulator");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", Arm.GetPositionCartesian);
        interfaceRequired->AddFunction("GetVelocityCartesian", Arm.GetVelocityCartesian);
        interfaceRequired->AddFunction("GetPositionJoint", Arm.GetPositionJoint);
        interfaceRequired->AddFunction("GetVelocityJoint", Arm.GetVelocityJoint);
        interfaceRequired->AddFunction("GetPeriodStatistics", Arm.GetPeriodStatistics);
        /*
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveDaVinciArmQtWidget::ErrorMessageEventHandler,
                                                this, "RobotErrorMsg");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveDaVinciArmQtWidget::StatusMessageEventHandler,
                                                this, "RobotStatusMsg");
        */
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
    executionResult = Arm.GetPositionCartesian(PositionCartesian);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Manipulator.GetPositionCartesian failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QFRPositionCartesianWidget->SetValue(PositionCartesian.Position());

    executionResult = Arm.GetVelocityCartesian(VelocityCartesian);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Manipulator.GetVelocityCartesian failed, \""
                                << executionResult << "\"" << std::endl;
    }

    executionResult = Arm.GetPositionJoint(PositionJoint);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Manipulator.GetPositionJoint failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QVPositionJointWidget->SetValue(PositionJoint.Position() * (180.0 / cmnPI));

    executionResult = Arm.GetVelocityJoint(VelocityJoint);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Manipulator.GetVelocityJoint failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QVVelocityJointWidget->SetValue(VelocityJoint.Velocity() * (180.0 / cmnPI));

    Arm.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsIntuitiveDaVinciArmQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsIntuitiveDaVinciArmQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    // Side by side for 3D position and timing
    QHBoxLayout * topLayout = new QHBoxLayout;
    mainLayout->addLayout(topLayout);

    // 3D position
    QFRPositionCartesianWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    topLayout->addWidget(QFRPositionCartesianWidget);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout();
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    topLayout->addLayout(timingLayout);

    // Vectors of values
    QGridLayout * gridLayout = new QGridLayout;
    mainLayout->addLayout(gridLayout);

    gridLayout->setSpacing(1);
    int row = 0;
    gridLayout->addWidget(new QLabel("Joint positions"), row, 0);
    QVPositionJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVPositionJointWidget, row, 1);
    row++;
    gridLayout->addWidget(new QLabel("Joint velocities"), row, 0);
    QVVelocityJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVVelocityJointWidget, row, 1);
    row++;

    // Messages
    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    mainLayout->addWidget(QTEMessages);

    setLayout(mainLayout);
    setWindowTitle("Manipulator");
    resize(sizeHint());

    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));
}

void mtsIntuitiveDaVinciArmQtWidget::ErrorMessageEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("red"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
}

void mtsIntuitiveDaVinciArmQtWidget::StatusMessageEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("black"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
}
