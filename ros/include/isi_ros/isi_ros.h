/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _isi_ros_h
#define _isi_ros_h

#include <string>
#include <list>

class mtsIntuitiveDaVinci;
class mtsROSBridge;

class isi_ros
{
public:
    isi_ros(mtsROSBridge * bridge,
            mtsROSBridge * tf_bridge,
            const std::string & ros_namespace,
            mtsIntuitiveDaVinci * daVinci);
    void Connect(void);

protected:
    std::string mBridgeName;
    std::string mTfBridgeName;
    std::string mNameSpace;
    mtsIntuitiveDaVinci * mDaVinci;

    typedef std::list<std::string> ArmsType;
    ArmsType Arms;

    typedef std::list<std::string> SUJsType;
    SUJsType SUJs;

    typedef std::list<std::string> ConsoleVoidEventsType;
    ConsoleVoidEventsType ConsoleVoidEvents;

    typedef std::list<std::string> ConsoleButtonEventsType;
    ConsoleButtonEventsType ConsoleButtonEvents;
};

#endif // _isi_ros_h
