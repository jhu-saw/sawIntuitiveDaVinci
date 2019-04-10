/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <isi_ros/isi_ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>

isi_ros::isi_ros(mtsROSBridge * bridge,
                 mtsROSBridge * tf_bridge,
                 const std::string & ros_namespace,
                 mtsIntuitiveDaVinci * daVinci):
    mNameSpace(ros_namespace),
    mDaVinci(daVinci)
{
    Arms.push_back("MTML1");
    Arms.push_back("MTMR1");
    Arms.push_back("PSM1");
    Arms.push_back("PSM2");
    Arms.push_back("PSM3");
    Arms.push_back("ECM1");

    SUJs.push_back("PSM1");
    SUJs.push_back("PSM2");
    SUJs.push_back("PSM3");
    SUJs.push_back("ECM1");

    ConsoleVoidEvents.push_back("HeadIn");
    ConsoleVoidEvents.push_back("HeadOut");
    ConsoleVoidEvents.push_back("ClutchQuickTap");
    ConsoleVoidEvents.push_back("CameraQuickTap");

    ConsoleButtonEvents.push_back("Standby");
    ConsoleButtonEvents.push_back("Ready");
    ConsoleButtonEvents.push_back("Clutch");
    ConsoleButtonEvents.push_back("Camera");
    ConsoleButtonEvents.push_back("MastersAsMice");
    ConsoleButtonEvents.push_back("FollowMode");

    mBridgeName = bridge->GetName();
    mTfBridgeName = tf_bridge->GetName();

    ArmsType::const_iterator armIter = Arms.begin();
    const ArmsType::const_iterator armsEnd = Arms.end();
    for (;
         armIter != armsEnd;
         ++armIter) {
        const std::string arm_namespace = ros_namespace + "/" + *armIter;
        bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (*armIter, "GetStateJoint",
             arm_namespace + "/state_joint_current");
        bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (*armIter, "GetPositionCartesian",
             arm_namespace + "/position_cartesian_current");
        bridge->AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (*armIter, "GetVelocityCartesian",
             arm_namespace + "/twist_body_current");

        // MTM specific
        if ((*armIter == "MTML1") || (*armIter == "MTMR1")) {
            bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                (*armIter, "GetStateGripper",
                 arm_namespace + "/state_gripper_current");
        } else if ((*armIter == "PSM1") || (*armIter == "PSM2") || (*armIter == "PSM3")) {
            bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                (*armIter, "GetStateJaw",
                 arm_namespace + "/state_jaw_current");
        }
        tf_bridge->Addtf2BroadcasterFromCommandRead(*armIter, "GetPositionCartesian");
    }

    SUJsType::const_iterator sujIter = SUJs.begin();
    const SUJsType::const_iterator sujsEnd = SUJs.end();
    for (;
         sujIter != sujsEnd;
         ++sujIter) {
        const std::string suj_namespace = ros_namespace + "/SUJ/" + *sujIter;
        bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (*sujIter, "GetStateJointSetup",
             suj_namespace + "/state_joint_current");
        bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (*sujIter, "GetPositionCartesianRCM",
             suj_namespace + "/position_cartesian_current");

        tf_bridge->Addtf2BroadcasterFromCommandRead(*sujIter, "GetPositionCartesianRCM");
    }

    ConsoleVoidEventsType::const_iterator voidEventsIter = ConsoleVoidEvents.begin();
    const ConsoleVoidEventsType::const_iterator voidEventsEnd = ConsoleVoidEvents.end();
    for (;
         voidEventsIter != voidEventsEnd;
         ++voidEventsIter) {
        std::string void_namespace = *voidEventsIter;
        // put everything lower case
        std::transform(void_namespace.begin(), void_namespace.end(),
                       void_namespace.begin(), tolower);

        bridge->AddPublisherFromEventVoid
            ("Console", *voidEventsIter, ros_namespace + "/console/" + void_namespace);
    }

    ConsoleButtonEventsType::const_iterator buttonEventsIter = ConsoleButtonEvents.begin();
    const ConsoleButtonEventsType::const_iterator buttonEventsEnd = ConsoleButtonEvents.end();
    for (;
         buttonEventsIter != buttonEventsEnd;
         ++buttonEventsIter) {
        std::string button_namespace = *buttonEventsIter;
        // put everything lower case
        std::transform(button_namespace.begin(), button_namespace.end(),
                       button_namespace.begin(), tolower);

        bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (*buttonEventsIter, "Button", ros_namespace + "/console/" + button_namespace);
    }
}

void isi_ros::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    ArmsType::const_iterator armIter = Arms.begin();
    const ArmsType::const_iterator armsEnd = Arms.end();
    for (;
         armIter != armsEnd;
         ++armIter) {
        componentManager->Connect(mBridgeName, *armIter,
                                  mDaVinci->GetName(), *armIter);
        componentManager->Connect(mTfBridgeName, *armIter,
                                  mDaVinci->GetName(), *armIter);
    }

    ConsoleButtonEventsType::const_iterator buttonEventsIter = ConsoleButtonEvents.begin();
    const ConsoleButtonEventsType::const_iterator buttonEventsEnd = ConsoleButtonEvents.end();
    for (;
         buttonEventsIter != buttonEventsEnd;
         ++buttonEventsIter) {
        componentManager->Connect(mBridgeName, *buttonEventsIter,
                                  mDaVinci->GetName(), *buttonEventsIter);
    }

    componentManager->Connect(mBridgeName, "Console",
                              mDaVinci->GetName(), "Console");
}
