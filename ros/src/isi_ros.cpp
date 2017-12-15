/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
}
