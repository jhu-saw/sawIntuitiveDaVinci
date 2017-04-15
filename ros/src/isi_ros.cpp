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

isi_ros::isi_ros(mtsROSBridge & bridge,
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

    mBridgeName = bridge.GetName();

    ArmsType::const_iterator iter = Arms.begin();
    const ArmsType::const_iterator end = Arms.end();
    for (;
         iter != end;
         ++iter) {
        const std::string arm_namespace = ros_namespace + "/" + *iter;
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (*iter, "GetStateJoint",
             arm_namespace + "/state_joint_current");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (*iter, "GetPositionCartesian",
             arm_namespace + "/position_cartesian_current");

        bridge.AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (*iter, "GetVelocityCartesian",
             arm_namespace + "/twist_body_current");

    }
}

void isi_ros::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    ArmsType::const_iterator iter = Arms.begin();
    const ArmsType::const_iterator end = Arms.end();
    for (;
         iter != end;
         ++iter) {
        componentManager->Connect(mBridgeName, *iter,
                                  mDaVinci->GetName(), *iter);
    }
}
