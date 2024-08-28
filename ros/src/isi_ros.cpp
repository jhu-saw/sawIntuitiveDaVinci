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
#include <cisst_ros_bridge/mtsROSBridge.h>

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
    std::list<std::string> console_void_events;
    console_void_events.push_back("console/head_in");
    console_void_events.push_back("console/head_out");
    console_void_events.push_back("console/clutch_quick_tap");
    console_void_events.push_back("console/camera_quick_tap");

    this->bridge_all_interfaces_provided(m_da_vinci_component_name,
                                         "", // namespace,
                                         m_publish_period_in_seconds,
                                         m_tf_period_in_seconds);

    for (const auto & event : console_void_events) {
        this->events_bridge().AddPublisherFromEventVoid("Console", event,
                                                        event);
    }
    this->m_connections.Add(this->events_bridge().GetName(), "Console",
                            m_da_vinci_component_name, "Console");
}
