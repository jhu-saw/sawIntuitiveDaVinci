/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <isi_ros/isi_ros.h>
#include <cisstCommon/cmnStrings.h>

isi_ros::isi_ros(const std::string & _da_vinci_component_name,
                 cisst_ral::node_ptr_t _node_handle,
                 const double _publish_period_in_seconds,
                 const double _tf_period_in_seconds):
    mts_ros_crtk_bridge_provided(_da_vinci_component_name + "_ros_bridge",
                                 _node_handle,
                                 _publish_period_in_seconds),
    m_da_vinci_component_name(_da_vinci_component_name),
    m_publish_period_in_seconds(_publish_period_in_seconds),
    m_tf_period_in_seconds(_tf_period_in_seconds)
{
    std::list<std::string> arms, sujs, mtms, mtms_buttons, arms_buttons, console_void_events, console_buttons;
        
    arms.push_back("MTML1");
    arms.push_back("MTMR1");
    arms.push_back("PSM1");
    arms.push_back("PSM2");
    arms.push_back("PSM3");
    arms.push_back("ECM1");

    sujs.push_back("PSM1");
    sujs.push_back("PSM2");
    sujs.push_back("PSM3");
    sujs.push_back("ECM1");

    mtms.push_back("MTML1");
    mtms.push_back("MTMR1");

    mtms_buttons.push_back("Select");
    mtms_buttons.push_back("Clutch");

    arms_buttons.push_back("FollowMode");

    console_void_events.push_back("HeadIn");
    console_void_events.push_back("HeadOut");
    console_void_events.push_back("ClutchQuickTap");
    console_void_events.push_back("CameraQuickTap");

    console_buttons.push_back("OperatorPresent");
    console_buttons.push_back("Clutch");
    console_buttons.push_back("Camera");
    console_buttons.push_back("MastersAsMice");
    console_buttons.push_back("FollowMode");

    this->bridge_all_interfaces_provided(m_da_vinci_component_name,
                                         "", // namespace,
                                         m_publish_period_in_seconds,
                                         m_tf_period_in_seconds);
#if 0
    for (const auto & _arm : _arms) {

        this->

        // arm events
        for (const auto & _button : _arms_buttons) { 
            bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                (_arm + _button, "Button",
                 _arm + "/" + cmnStringToUnderscoreLower(_button));
        }

    }

    // MTM events
    _armsType::const_iterator mtmIter = _mtms.begin();
    const _armsType::const_iterator mtmsEnd = _mtms.end();
    for (;
         mtmIter != mtmsEnd;
         ++mtmIter) {
        const std::string arm_namespace = *mtmIter;
        ButtonEventsType::const_iterator buttonEventsIter = _mtmsButtonEvents.begin();
        const ButtonEventsType::const_iterator buttonEventsEnd = _mtms_buttons.end();
        for (;
             buttonEventsIter != buttonEventsEnd;
             ++buttonEventsIter) {
            bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                (*mtmIter + *buttonEventsIter, "Button",
                 arm_namespace + "/" + cmnStringToUnderscoreLower(*buttonEventsIter));
        }
    }

    _sujsType::const_iterator sujIter = _sujs.begin();
    const _sujsType::const_iterator sujsEnd = _sujs.end();
    for (;
         sujIter != sujsEnd;
         ++sujIter) {
        const std::string suj_namespace = "SUJ/" + *sujIter;
        bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (*sujIter, "GetStateJointSetup",
             suj_namespace + "/measured_js");
        bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (*sujIter, "GetPositionCartesianRCM",
             suj_namespace + "/measured_cp");

        tf_bridge->Addtf2BroadcasterFromCommandRead(*sujIter, "GetPositionCartesianRCM");
    }

    VoidEventsType::const_iterator voidEventsIter = _console_void_events.begin();
    const VoidEventsType::const_iterator voidEventsEnd = _console_void_events.end();
    for (;
         voidEventsIter != voidEventsEnd;
         ++voidEventsIter) {
        bridge->AddPublisherFromEventVoid
            ("Console", *voidEventsIter,
             "console/" + cmnStringToUnderscoreLower(*voidEventsIter));
    }

    ButtonEventsType::const_iterator buttonEventsIter = _console_buttons.begin();
    const ButtonEventsType::const_iterator buttonEventsEnd = _console_buttons.end();
    for (;
         buttonEventsIter != buttonEventsEnd;
         ++buttonEventsIter) {
        bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (*buttonEventsIter, "Button",
             "console/" + cmnStringToUnderscoreLower(*buttonEventsIter));
    }
#endif
}
