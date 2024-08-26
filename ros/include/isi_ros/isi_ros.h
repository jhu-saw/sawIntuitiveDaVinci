/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _isi_ros_h
#define _isi_ros_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

class isi_ros: public mts_ros_crtk_bridge_provided
{
public:
    isi_ros(const std::string & _da_vinci_component_name,
            cisst_ral::node_ptr_t _node_handle,
            const double _publish_period_in_seconds = 20.0 * cmn_ms,
            const double _tf_period_in_seconds = 20.0 * cmn_ms);

protected:
    std::string m_da_vinci_component_name;
    double m_publish_period_in_seconds;
    double m_tf_period_in_seconds;
};

#endif // _isi_ros_h
