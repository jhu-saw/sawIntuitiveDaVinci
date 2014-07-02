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


#ifndef _mtsIntuitiveDaVinciArmQtWidget_h
#define _mtsIntuitiveDaVinciArmQtWidget_h

#include <cisstMultiTask/mtsComponent.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>

#include <cisstVector/vctQtWidgetFrame.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>



#include <QtCore>
#include <QtGui>


class mtsIntuitiveDaVinciArmQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveDaVinciArmQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveDaVinciArmQtWidget() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalAppendMessage(QString);
    void SignalSetColor(QColor);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotTextChanged(void);

private:
    //! setup GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

protected:
    struct ArmStruct {
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionRead GetVelocityCartesian;
        mtsFunctionRead GetPositionJoint;
        mtsFunctionRead GetVelocityJoint;
        mtsFunctionRead GetPeriodStatistics;
    } Arm;

private:
    prmPositionCartesianGet PositionCartesian;
    prmVelocityCartesianGet VelocityCartesian;
    prmPositionJointGet PositionJoint;
    prmVelocityJointGet VelocityJoint;

    vctQtWidgetFrameDoubleRead * QFRPositionCartesianWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVPositionJointWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVVelocityJointWidget;

    // Timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    // Messages
    void ErrorMessageEventHandler(const std::string & message);
    void StatusMessageEventHandler(const std::string & message);
    QTextEdit * QTEMessages;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveDaVinciArmQtWidget);

#endif // _mtsIntuitiveDaVinciArmQtWidget_h
