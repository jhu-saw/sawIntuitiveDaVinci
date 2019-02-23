/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Nicolas Padoy
  Created on: 2010-04-06

  (C) Copyright 2010-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveDaVinci_h
#define _mtsIntuitiveDaVinci_h

// cisst library headers
#include <cisstMultiTask/mtsMacros.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsFunctionVoid.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>

// Always include last
#include <sawIntuitiveDaVinci/sawIntuitiveDaVinciExport.h>

/*!
  \file
  \brief Declaration of mtsIntuitiveDaVinci
  \ingroup sawIntuitiveDaVinci
*/

/*!
  \ingroup sawIntuitiveDaVinci

  This class wraps the ISI da Vinci BBAPI API (read-write) and is
  intended to provide a high-level interface to the da Vinci.

*/

// forward declarations
class mtsIntuitiveDaVinci;

namespace mtsIntuitiveDaVinciUtilities {
    void StreamCallbackInternal(void * userData);
    void EventCallbackInternal(int manipulatorId,
                               int eventId,
                               int* arguments,
                               void * userData);
};


class CISST_EXPORT mtsIntuitiveDaVinci: public mtsTaskPeriodic { //mtsTaskFromSignal {

    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_WARNING);

    friend void mtsIntuitiveDaVinciUtilities::StreamCallbackInternal(void * userData);
    friend void mtsIntuitiveDaVinciUtilities::EventCallbackInternal(int manipulatorId,
                                                                    int eventId,
                                                                    int* arguments,
                                                                    void * userData);
 public:

    /*! Constructor.*/
     mtsIntuitiveDaVinci(const std::string & name, unsigned int rateInHz);

    /*! Default destructor. Does nothing. */
    ~mtsIntuitiveDaVinci();

    void SetSourceHost(const std::string _ipaddress, const unsigned int _port = 5002, const unsigned int _password = 0x1111);
    void SetSourceLogFile(const std::string _filename);
    /*! Set filename to log the API to in the bin format */
    void SetOutputLogFile(const std::string filename);
    /*! Start logging to bin file. */
    void StartLogging(void);
    /*! Stop logging to bin file. */
    void StopLogging(void);

    void Configure(const std::string & CMN_UNUSED(filename) = "") {};
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    /*! Provided for compatibility with mtsIntuitiveDaVinci and C example */
    inline bool IsAPIConnected(void) const {
        return this->Connected;
    }

    /*! Type of manipulator.  Note that indices are contiguous by type
      of manipulator. */
    typedef enum {
        MTML1 = 0,          /*! Master Left 1*/
        MTMR1,              /*! Master Right 1*/
        MTML2,              /*! Master Left 2*/
        MTMR2,              /*! Master Right 2*/
        PSM1,               /*! Patient side 1 */
        PSM2,               /*! Patient side 2 */
        PSM3,               /*! Patient side 3 */
        ECM1,               /*! Endoscopic camera */
        CONSOLE1,           /*! Console */
        CONSOLE2,           /*! Dual Console */
        CORE,               /*! In Si system events are generated from Core as well */
        NUMBER_MANIPULATORS /*! Number of manipulators */
    } ManipulatorIndexType;

    typedef enum {
        MTM_TYPE = 0,                    /*! Master Tool Manipulator */
        PSM_TYPE,                        /*! Patient Side Manipulator */
        ECM_TYPE,                        /*! Endoscopic Camera Manipulator */
        CONSOLE_TYPE,                    /*! Surgeon-Side Console */
        NUMBER_MANIPULATOR_TYPES         /*! Number of manipulator types */
    } ManipulatorType;

 protected:

    /*! Class to contain the data common to all arms */
    class ArmData {
    public:
        ArmData(void);
        mtsStateTable * StateTable;
        mtsInterfaceProvided * ProvidedInterface;
        mtsFunctionVoid DataUpdated;
        mtsFloat DeviceTimestamp;
        prmPositionCartesianGet PositionCartesian;
        prmVelocityCartesianGet VelocityCartesian;
        prmStateJoint StateJoint;
    };

    /*! Class to contain the data specific to the master arms */
    class MasterArmData: public ArmData {
    public:
        MasterArmData(void);
        mtsInterfaceProvided * SelectEventProvidedInterface;
        prmStateJoint StateGripper;
        mtsFunctionWrite Select;
        bool Selected;
        static const double SelectAngle;
        mtsInterfaceProvided * ClutchEventProvidedInterface;
        mtsFunctionWrite Clutch;
        bool ClutchRestAngleNeedsUpdate;
        double ClutchRestAngle;
        bool Clutched;
        static const double ClutchAngle;
    };

    /*! Class to contain the data specific to slave arms */
    class SlaveArmData: public ArmData {
    public:
        SlaveArmData(void);
        prmStateJoint StateJaw;
        prmPositionCartesianGet PositionCartesianRCM;
        prmPositionCartesianGet PositionCartesianSetup;
        prmStateJoint StateSUJ;
    };

    /*! Class to contain data specific to the console */
    class ConsoleData
    {
    public:
        ConsoleData(void);
        mtsInterfaceProvided * ProvidedInterface;
        mtsFunctionVoid HeadIn;
        mtsFunctionVoid HeadOut;
        mtsFunctionVoid ClutchQuickTap;
        mtsFunctionVoid CameraQuickTap;

        mtsInterfaceProvided * StandbyProvidedInterface;
        mtsFunctionWrite Standby;

        mtsInterfaceProvided * ReadyProvidedInterface;
        mtsFunctionWrite Ready;

        mtsInterfaceProvided * ClutchProvidedInterface;
        mtsFunctionWrite Clutch;
        bool Clutched;

        mtsInterfaceProvided * CameraProvidedInterface;
        mtsFunctionWrite Camera;

        mtsInterfaceProvided * MastersAsMiceProvidedInterface;
        mtsFunctionWrite MastersAsMice;
        bool MastersAsMiced;

        mtsInterfaceProvided * FollowModeProvidedInterface;
        mtsFunctionWrite FollowMode;
    };

    class EventData {
    public:
        EventData(void);
        mtsInterfaceProvided * ProvidedInterface;
        vctDynamicVector<mtsFunctionWrite *> WriteFunctions;
        vctDynamicVector<std::string> EventNames;
    };

    /*! Info for all arms, this container is the primary one */
    ArmData * Arms[(ECM1 - MTML1) + 1];

    /*! Info for all master arms, different way to access masters only */
    MasterArmData * MasterArms[(MTMR2 - MTML1) + 1];

    /*! Info for all slave arms, including camera */
    SlaveArmData * SlaveArms[(ECM1 - PSM1) + 1];

    /*! Camera arm ECM1 */
    SlaveArmData * CameraArms[(ECM1 - ECM1) + 1];

    /*! Console data */
    ConsoleData Console;
    ConsoleData * Consoles[(CONSOLE2 - CONSOLE1) + 1];

    /*! Event data */
    EventData Events;

    /*! Connect to the daVinci system. */
    bool Connect(void);

    /*! Disconnect from the daVinci system. */
    bool Disconnect(void);

    bool ConfigureStream(void);
    void StreamCallback(void);

    bool ConfigureEvents(void);
    void EventCallback(ManipulatorIndexType index, int eventId, int* eventArgs);

    bool StartStream(void);

    void StopStream(void);

    /*! Log system information */
    void LogSystemConfiguration(cmnLogLevel logLevel = CMN_LOG_LEVEL_INIT_VERBOSE) const;
    void LogManipulatorConfiguration(ManipulatorIndexType index,
                                     cmnLogLevel logLevel = CMN_LOG_LEVEL_INIT_VERBOSE) const;
    void LogToolConfiguration(ManipulatorIndexType index,
                              cmnLogLevel logLevel = CMN_LOG_LOD_INIT_VERBOSE) const;
    void LogManipulatorsAndToolsConfiguration(cmnLogLevel logLevel = CMN_LOG_LOD_INIT_VERBOSE) const;

 public:
    /*! Conversion from manipulator index to string */
    static const std::string & ManipulatorIndexToString(ManipulatorIndexType manipulatorIndex);

    /*! Get number of joints per arm */
    static size_t GetNumberOfJoints(ManipulatorIndexType manipulatorIndex);

    /*! Get number of setup joints for the specified manipulator index */
    static size_t GetNumberOfSetupJoints(ManipulatorIndexType manipulatorIndex);

    /*! Get manipulator type from manipulator index */
    static ManipulatorType GetManipulatorType(ManipulatorIndexType manipulatorIndex);

    inline vctDynamicVector<std::string> GetEventNames(void) {
        return this->Events.EventNames;
    }

 protected:
    /*! Setup all interfaces and populate them with commands and events */
    //@{
    void SetupEventInterfaces(void);
    void SetupArmsInterfaces(void);
    void SetupMastersInterfaces(void);
    void SetupSlavesInterfaces(void);
    void SetupCameraInterfaces(void);
    void SetupConsoleInterfaces(void);
    void SetupAllInterfaces(void);
    //@}

    inline bool IsMTM(const ManipulatorIndexType index) const {
        return ((index == MTML1) || (index == MTMR1) || (index == MTML2) || (index == MTMR2));
    }

    inline bool IsPSM(const ManipulatorIndexType index) const {
        return ((index == PSM1) || (index == PSM2) || (index == PSM3));
    }

    inline bool IsECM(const ManipulatorIndexType index) const {
        return (index == ECM1);
    }

    bool Connected;

    unsigned int RateInHz;
    std::string IPAddress;
    unsigned int Port;
    unsigned int Password;
    std::string SourceLogFileName;
    bool UseLogFile;
    std::string OutputLogFileName;
};


CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveDaVinci);


#endif // _mtsIntuitiveDaVinci_h
