/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Nicolas Padoy
  Created on: 2010-04-06

  (C) Copyright 2010-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveDaVinci/mtsIntuitiveDaVinci.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaSleep.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmJointType.h>

#include <isi_api.h>


namespace mtsIntuitiveDaVinciUtilities
{
    const ISI_UINT ALL_DOFS[daVinci::NUM_MAX_JOINTS] = { 0, 1, 2, 3, 4, 5, 6, 7 };

    ISI::MANIP_INDEX ManipulatorIndexToISI(mtsIntuitiveDaVinci::ManipulatorIndexType index)
    {
        switch (index) {
        case mtsIntuitiveDaVinci::MTML1:
            return daVinci::MTML1;
        case mtsIntuitiveDaVinci::MTMR1:
            return daVinci::MTMR1;
        case mtsIntuitiveDaVinci::MTML2:
            return daVinci::MTML2;
        case mtsIntuitiveDaVinci::MTMR2:
            return daVinci::MTMR2;
        case mtsIntuitiveDaVinci::PSM1:
            return daVinci::PSM1;
        case mtsIntuitiveDaVinci::PSM2:
            return daVinci::PSM2;
        case mtsIntuitiveDaVinci::PSM3:
            return daVinci::PSM3;
        case mtsIntuitiveDaVinci::ECM1:
            return daVinci::ECM; // Black Box API dropped the number
        case mtsIntuitiveDaVinci::CONSOLE1:
            return daVinci::CONSOLE1;
        case mtsIntuitiveDaVinci::CONSOLE2:
            return daVinci::CONSOLE2;
        case mtsIntuitiveDaVinci::CORE:
            return daVinci::CORE;
        default:
            CMN_LOG_RUN_WARNING << "mtsIntuitiveDaVinciUtilities::ManipulatorIndexToISI: index out of range" << std::endl;
        }
        return daVinci::NUM_MANIPS;
    }


    mtsIntuitiveDaVinci::ManipulatorIndexType ManipulatorIndexFromISI(ISI::MANIP_INDEX index)
    {
        switch (index) {
        case daVinci::MTML1:
            return mtsIntuitiveDaVinci::MTML1;
        case daVinci::MTMR1:
            return mtsIntuitiveDaVinci::MTMR1;
        case daVinci::MTML2:
            return mtsIntuitiveDaVinci::MTML2;
        case daVinci::MTMR2:
            return mtsIntuitiveDaVinci::MTMR2;
        case daVinci::PSM1:
            return mtsIntuitiveDaVinci::PSM1;
        case daVinci::PSM2:
            return mtsIntuitiveDaVinci::PSM2;
        case daVinci::PSM3:
            return mtsIntuitiveDaVinci::PSM3;
        case daVinci::ECM:
            return mtsIntuitiveDaVinci::ECM1; // Black Box API dropped the number
        case daVinci::CONSOLE1:
            return mtsIntuitiveDaVinci::CONSOLE1;
        case daVinci::CONSOLE2:
            return mtsIntuitiveDaVinci::CONSOLE2;
        case daVinci::CORE:
            return mtsIntuitiveDaVinci::CORE;
        default:
            CMN_LOG_RUN_WARNING << "mtsIntuitiveDaVinciUtilities::ManipulatorIndexToISI: index out of range" << std::endl;
        }
        return mtsIntuitiveDaVinci::NUMBER_MANIPULATORS;
    }


    std::string StatusToString(ISI_STATUS status)
    {
        switch (status) {
        case ISI_SUCCESS:
            return "ISI_SUCCESS";
        case ISI_UNKNOWN_ERROR:
            return "ISI_UNKNOWN_ERROR";
        case ISI_INVALID_INPUT:
            return "ISI_INVALID_INPUT";
        case ISI_INVALID_OPERATION:
            return "ISI_INVALID_OPERATION";
        case ISI_INVALID_CONNECTION:
            return "ISI_INVALID_CONNECTION";
        case ISI_NULL_POINTER_ERROR:
            return "ISI_NULL_POINTER_ERROR";
        case ISI_COMMUNICATION_ERROR:
            return "ISI_COMMUNICATION_ERROR";
        case ISI_AUTHENTICATION_ERROR:
            return "ISI_AUTHENTICATION_ERROR";
        default:
            CMN_LOG_RUN_ERROR << "mtsIntuitiveDaVinciUtilities::StatusToString: unknown status code \"" << status << "\"" << std::endl;
        }
        return "Unknown error code";
    }


    std::string SystemConfigurationToString(const ISI_SYSTEM_CONFIG & systemConfiguration)
    {
        std::stringstream outputStream;
        outputStream << "System name: " << systemConfiguration.system_name << std::endl
                     << "System version: " << systemConfiguration.system_version << std::endl
                     << "Library version: " << systemConfiguration.library_version;
        return outputStream.str();
    }


    void ISICALLBACK StreamCallback(void * userData)
    {
        StreamCallbackInternal(userData);
    }


    void StreamCallbackInternal(void * userData)
    {
        // convert userData pointer
        mtsIntuitiveDaVinci * instance =
            reinterpret_cast<mtsIntuitiveDaVinci *>(userData);
        CMN_ASSERT(instance);
        instance->StreamCallback();
    }


    void ISICALLBACK EventCallback(ISI::MANIP_INDEX isiManipulatorIndex,
                                   ISI_EVENT_ID eventId,
                                   ISI_INT (arguments[ISI_NUM_EVENT_ARGS]),
                                   void * userData)
    {
        EventCallbackInternal(isiManipulatorIndex, eventId, arguments, userData);
    }


    void EventCallbackInternal(int isiManipulatorIndex,
                               int eventId,
                               int * eventArgs,
                               void * userData)
    {
        // convert userData pointer
        mtsIntuitiveDaVinci * instance =
            reinterpret_cast<mtsIntuitiveDaVinci *>(userData);
        CMN_ASSERT(instance);
        // convert ISI manipulator index
        mtsIntuitiveDaVinci::ManipulatorIndexType manipulatorIndex =
            mtsIntuitiveDaVinciUtilities::ManipulatorIndexFromISI(ISI::MANIP_INDEX(isiManipulatorIndex));
        instance->EventCallback(manipulatorIndex, eventId, eventArgs);
    }


    // Convert ISI_TRANSFORM to vctFrm3
    void FrameFromISI(const ISI_TRANSFORM & input, vctFrm3 & output)
    {
        // ISI_TRANSFORM.pos is a struct with 3 floats, x, y, and z
        output.Translation().Assign(vctFixedSizeConstVectorRef<float, 3, 1>(&(input.pos.x)));
#if !CISST_USE_SI_UNITS
        output.Translation().Multiply(1000.0); // ISI meters to cisst millimeters
#endif
        // rotation
        output.Rotation().Column(0).Assign(input.rot.row0.x, input.rot.row1.x, input.rot.row2.x);
        output.Rotation().Column(1).Assign(input.rot.row0.y, input.rot.row1.y, input.rot.row2.y);
        output.Rotation().Column(2).Assign(input.rot.row0.z, input.rot.row1.z, input.rot.row2.z);
    }

    // Convert vctFrm3 to ISI_TRANSFORM
    void FrameToISI(const vctFrm3 & input, ISI_TRANSFORM & output)
    {
        output.pos.x = static_cast<ISI_FLOAT>(input.Translation().X() / 1000.0);
        output.pos.y = static_cast<ISI_FLOAT>(input.Translation().Y() / 1000.0);
        output.pos.z = static_cast<ISI_FLOAT>(input.Translation().Z() / 1000.0);

        output.rot.row0.x = static_cast<ISI_FLOAT>(input.Rotation().Element(0, 0));
        output.rot.row0.y = static_cast<ISI_FLOAT>(input.Rotation().Element(0, 1));
        output.rot.row0.z = static_cast<ISI_FLOAT>(input.Rotation().Element(0, 2));
        output.rot.row1.x = static_cast<ISI_FLOAT>(input.Rotation().Element(1, 0));
        output.rot.row1.y = static_cast<ISI_FLOAT>(input.Rotation().Element(1, 1));
        output.rot.row1.z = static_cast<ISI_FLOAT>(input.Rotation().Element(1, 2));
        output.rot.row2.x = static_cast<ISI_FLOAT>(input.Rotation().Element(2, 0));
        output.rot.row2.y = static_cast<ISI_FLOAT>(input.Rotation().Element(2, 1));
        output.rot.row2.z = static_cast<ISI_FLOAT>(input.Rotation().Element(2, 2));
    }

} // end of namespace mtsIntuitiveDaVinciUtilities


mtsIntuitiveDaVinci::ArmData::ArmData(void):
    StateTable(0),
    ConfigurationStateTable(0),
    ProvidedInterface(0),
    FollowModeProvidedInterface(0)
{}


const double mtsIntuitiveDaVinci::MTMArmData::SelectAngle = 0.2;
const double mtsIntuitiveDaVinci::MTMArmData::ClutchAngle = 0.7;

mtsIntuitiveDaVinci::MTMArmData::MTMArmData(void):
    SelectProvidedInterface(0),
    Selected(false),
    ClutchProvidedInterface(0),
    ClutchRestAngleNeedsUpdate(false),
    Clutched(false)
{}


mtsIntuitiveDaVinci::PSMArmData::PSMArmData(void)
{}

mtsIntuitiveDaVinci::ConsoleData::ConsoleData(void):
    ProvidedInterface(0),
    OperatorPresentProvidedInterface(0),
    ClutchProvidedInterface(0),
    Clutched(false),
    CameraProvidedInterface(0),
    MastersAsMiceProvidedInterface(0),
    MastersAsMiced(false)
{}

mtsIntuitiveDaVinci::EventData::EventData(void):
    ProvidedInterface(0)
{}

CMN_IMPLEMENT_SERVICES(mtsIntuitiveDaVinci);


mtsIntuitiveDaVinci::mtsIntuitiveDaVinci(const std::string & name, unsigned int rateInHz):
    mtsTaskPeriodic(name, 1.0 * cmn_ms), //TaskFromSignal(name),
    Connected(false),
    RateInHz(rateInHz)
{
    this->SetSourceHost("10.0.0.5");
    this->SetupAllInterfaces();
}


mtsIntuitiveDaVinci::~mtsIntuitiveDaVinci()
{
}

void mtsIntuitiveDaVinci::SetSourceHost(const std::string _ipaddress, const unsigned int _port, const unsigned int _password)
{
    UseLogFile = false;
    IPAddress = _ipaddress;
    Port = _port;
    Password = _password;
}

void mtsIntuitiveDaVinci::SetSourceLogFile(const std::string _filename)
{
    UseLogFile = true;
    SourceLogFileName = _filename;
}

void mtsIntuitiveDaVinci::SetOutputLogFile(const std::string filename)
{
    OutputLogFileName = filename;
}

void mtsIntuitiveDaVinci::StartLogging(void)
{
    isi_start_logging(OutputLogFileName.c_str());
}

void mtsIntuitiveDaVinci::StopLogging(void)
{
    isi_stop_logging();
}

void mtsIntuitiveDaVinci::Startup(void)
{
    unsigned int attemptNumber = 1;
    while ((attemptNumber <= 5)
           && (!this->Connected)) {
        this->Connect();
        if (!this->Connected) {
            CMN_LOG_CLASS_INIT_WARNING << "Startup: attempt number " << attemptNumber << " to connect failed for \""
                                       << this->GetName() << "\"" << std::endl;
        }
        attemptNumber++;
        osaSleep(1.0 * cmn_s);
    }
    // check connection
    if (!this->Connected) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: can not connect \"" << this->GetName() << "\"" << std::endl;
        return;
    }
    // provide some log
    this->LogSystemConfiguration();
    this->LogManipulatorsAndToolsConfiguration();
    // configure and start stream
    this->ConfigureStream();
    this->StartStream();
    // configure events
    this->ConfigureEvents();
}


void mtsIntuitiveDaVinci::Run(void)
{
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
}


void mtsIntuitiveDaVinci::Cleanup(void)
{
    this->StopStream();
    this->Disconnect();
}


bool mtsIntuitiveDaVinci::Connect(void)
{
    ISI_STATUS status;
    if (!UseLogFile) {
        status = isi_connect_ex(IPAddress.c_str(), Port, Password);
    }
    else {
        status = isi_connect_log(SourceLogFileName.c_str());
    }
    if (status != ISI_SUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "Connect: connection failed for \"" << this->GetName()
                                 << "\", status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        this->Connected = false;
    } else {
        CMN_LOG_CLASS_INIT_DEBUG << "Connect: connection succeedeed for \"" << this->GetName()
                                 << "\", status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        this->Connected = true;
    }
    return this->Connected;
}


bool mtsIntuitiveDaVinci::Disconnect(void)
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "Disconnect: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return true;
    }
    // API calls
    isi_disconnect();
    this->Connected = false;
    return this->Connected;
}


bool mtsIntuitiveDaVinci::ConfigureStream(void)
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "ConfigureStream: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return false;
    }
    // API calls
    ISI_STATUS status;
    status = isi_subscribe_all_stream_fields();
    if (status != ISI_SUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureStream: subscribe all stream fields failed, status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        return false;
    }

    // register callback
    status = isi_set_stream_callback(mtsIntuitiveDaVinciUtilities::StreamCallback, this);
    if (status != ISI_SUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureStream: failed to set stream callback, status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        return false;
    }

    return true;
}


void mtsIntuitiveDaVinci::StreamCallback(void)
{
    // API calls
    mtsIntuitiveDaVinci::ManipulatorIndexType index;
    ISI::MANIP_INDEX isiIndex;
    ISI_STATUS status;
    ISI_STREAM_FIELD streamData;
    vctDynamicConstVectorRef<float> jointRef;
    size_t numberOfJoints;
    bool eventCriterion;
    ArmData * arm;
    MTMArmData * _mtm;
    PSMArmData * _psm;

    bool atLeastOneMTMSelected = false;
    prmEventButton buttonPayload;
    buttonPayload.SetValid(true);
    buttonPayload.SetTimestamp(mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime());

    // subscribe to the API stream
    for (index = MTML1;
         index < CONSOLE1;
         index = static_cast<ManipulatorIndexType>(index + 1)) {
        // get local indices and pointers setup
        isiIndex = mtsIntuitiveDaVinciUtilities::ManipulatorIndexToISI(index);
        arm = this->Arms[index];
        arm->StateTable->Start();
        // joint values for all manipulators
        status = isi_get_stream_field(isiIndex, ISI_JOINT_VALUES, &streamData);
        if (status == ISI_SUCCESS) {
            arm->DeviceTimestamp.Assign(static_cast<mtsFloat>(streamData.timestamp));
        }
        if (status != ISI_SUCCESS) {
            CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_JOINT_VALUES, manipulator \""
                                    << ManipulatorIndexToString(index) << "\", status: "
                                    << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        } else {
            numberOfJoints = GetNumberOfJoints(index);
            // check size of data
            if (streamData.count != numberOfJoints) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_JOINT_VALUES, manipulator \""
                                        << ManipulatorIndexToString(index) << "\", expected "
                                        << numberOfJoints << ", received "
                                        << streamData.count << std::endl;
            } else {
                arm->m_measured_js.Valid() = true;
                // save the joint values
                if (IsECM(index)) {
                    jointRef.SetRef(numberOfJoints, streamData.data);
                    arm->m_measured_js.Position().Assign(jointRef);
                } else if (IsMTM(index)) {
                    _mtm = reinterpret_cast<MTMArmData *>(arm);
                    jointRef.SetRef(numberOfJoints - 1, streamData.data);
                    _mtm->m_measured_js.Position().Assign(jointRef);
                    _mtm->m_gripper_measured_js.Position().at(0) = streamData.data[numberOfJoints - 1];
                    _mtm->m_gripper_measured_js.Valid() = true;

                    // compute angle on last joint to trigger select event
                    eventCriterion = (_mtm->m_gripper_measured_js.Position().at(0) < MTMArmData::SelectAngle);
                    if (eventCriterion && !_mtm->Selected) {
                        atLeastOneMTMSelected = true;
                        buttonPayload.SetType(prmEventButton::PRESSED);
                        _mtm->Select(buttonPayload);
                        _mtm->Selected = true;
                        _mtm->ProvidedInterface->SendStatus("Gripper closed");
                    } else if (!eventCriterion && _mtm->Selected) {
                        buttonPayload.SetType(prmEventButton::RELEASED);
                        _mtm->Select(buttonPayload);
                        _mtm->Selected = false;
                        _mtm->ProvidedInterface->SendStatus("Gripper opened");
                    }
                    // clutch on masters makes sense only when master clutch is on
                    if (this->Console.Clutched) {
                        // update clutch rest angle if needed
                        if (_mtm->ClutchRestAngleNeedsUpdate) {
                            _mtm->ClutchRestAngle = _mtm->m_measured_js.Position()[numberOfJoints - 2];
                            _mtm->ClutchRestAngleNeedsUpdate = false;
                        }
                        // compute angle on before last joint to trigger select event
                        eventCriterion = (fabs((_mtm->m_measured_js.Position()[numberOfJoints - 2])
                                               - _mtm->ClutchRestAngle) > MTMArmData::ClutchAngle);
                        if (eventCriterion && !_mtm->Clutched) {
                            buttonPayload.SetType(prmEventButton::PRESSED);
                            _mtm->Clutch(buttonPayload);
                            _mtm->Clutched = true;
                            _mtm->ProvidedInterface->SendStatus("Clutched");
                        } else if (!eventCriterion && _mtm->Clutched) {
                            buttonPayload.SetType(prmEventButton::RELEASED);
                            _mtm->Clutch(buttonPayload);
                            _mtm->Clutched = false;
                            _mtm->ProvidedInterface->SendStatus("Unclutched");
                        }
                    }
                } else if (IsPSM(index)) {
                    _psm = reinterpret_cast<PSMArmData *>(arm);
                    jointRef.SetRef(numberOfJoints - 1, streamData.data);
                    _psm->m_measured_js.Position().Assign(jointRef);
                    _psm->m_jaw_measured_js.Position().at(0) = streamData.data[numberOfJoints - 1];
                    _psm->m_jaw_measured_js.Valid() = true;
                }
            }
        }

        // based on joints, figure out if we are entering MaM
        if (this->Console.Clutched // needs to be clutched
            && atLeastOneMTMSelected // needs to have at least one arm clutched in this iteration
            && !this->Console.MastersAsMiced // needs not to be in MaM already
            && this->MTMArms[0]->Selected // needs both arms to be clutched
            && this->MTMArms[1]->Selected) {
            buttonPayload.SetType(prmEventButton::PRESSED);
            this->Console.MastersAsMice(buttonPayload);
            this->Console.MastersAsMiced = true;
        }

        // tip transform for all manipulators
        status = isi_get_stream_field(isiIndex, ISI_TIP_TRANSFORM, &streamData);
        if (status != ISI_SUCCESS) {
            CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_TIP_TRANSFORM, manipulator  \""
                                    << ManipulatorIndexToString(index) << "\", status: "
                                    << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        } else {
            // check size of data, 4 vectors
            if (streamData.count != 12) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_TIP_TRANSFORM, manipulator \""
                                        << ManipulatorIndexToString(index) << "\", expected 12, received "
                                        << streamData.count << std::endl;
                arm->m_measured_cp.Valid() = false;
            } else {
                ISI_TRANSFORM * isiTransform = reinterpret_cast<ISI_TRANSFORM *>(streamData.data);
                mtsIntuitiveDaVinciUtilities::FrameFromISI(*isiTransform,
                                                           arm->m_measured_cp.Position());
                arm->m_measured_cp.Valid() = true;
            }
        }

        // joint velocities for all manipulators
        status = isi_get_stream_field(isiIndex, ISI_JOINT_VELOCITY, &streamData);
        if (status != ISI_SUCCESS) {
            CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_JOINT_VELOCITY, manipulator  \""
                                    << ManipulatorIndexToString(index) << "\", status: "
                                    << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        } else {
            numberOfJoints = GetNumberOfJoints(index);
            // check size of data
            if (streamData.count != numberOfJoints) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_JOINT_VELOCITY, manipulator \""
                                        << ManipulatorIndexToString(index) << "\", expected "
                                        << numberOfJoints << ", received "
                                        << streamData.count << std::endl;
            } else {
                // save the joint values
                vctDynamicConstVectorRef<float> jointVel;
                if (IsECM(index)) {
                    jointVel.SetRef(numberOfJoints, streamData.data);
                    arm->m_measured_js.Velocity().Assign(jointVel);
                } else if (IsMTM(index)) {
                    _mtm = reinterpret_cast<MTMArmData *>(arm);
                    jointVel.SetRef(numberOfJoints - 1, streamData.data);
                    _mtm->m_measured_js.Velocity().Assign(jointVel);
                    _mtm->m_gripper_measured_js.Velocity().at(0) = streamData.data[numberOfJoints - 1];
                } else if (IsPSM(index)) {
                    _psm = reinterpret_cast<PSMArmData *>(arm);
                    jointVel.SetRef(numberOfJoints - 1, streamData.data);
                    _psm->m_measured_js.Velocity().Assign(jointVel);
                    _psm->m_jaw_measured_js.Velocity().at(0) = streamData.data[numberOfJoints - 1];
                }
            }
        }

        // joint torques for all manipulators
        status = isi_get_stream_field(isiIndex, ISI_JOINT_TORQUE, &streamData);
        if (status != ISI_SUCCESS) {
            CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_JOINT_TORQUE, manipulator  \""
                                    << ManipulatorIndexToString(index) << "\", status: "
                                    << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        } else {
            if (IsMTM(index)) {
                numberOfJoints = GetNumberOfJoints(index) - 1; // MTMs don't report effort on gripper
            } else {
                numberOfJoints = GetNumberOfJoints(index);
            }
            // check size of data
            if (streamData.count != numberOfJoints) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_JOINT_TORQUE, manipulator \""
                                        << ManipulatorIndexToString(index) << "\", expected "
                                        << numberOfJoints << ", received "
                                        << streamData.count << std::endl;
                arm->m_measured_js.Valid() = false;
            } else {
                // save the joint values
                vctDynamicConstVectorRef<float> jointTorque;
                if (IsECM(index) || IsMTM(index)) {
                    jointTorque.SetRef(numberOfJoints, streamData.data);
                    arm->m_measured_js.Effort().Assign(jointTorque);
                } else if (IsPSM(index)) {
                    _psm = reinterpret_cast<PSMArmData *>(arm);
                    jointTorque.SetRef(numberOfJoints - 1, streamData.data);
                    _psm->m_measured_js.Effort().Assign(jointTorque);
                    _psm->m_jaw_measured_js.Effort().at(0) = streamData.data[numberOfJoints - 1];
                }
            }
        }

        // tip linear velocity for all manipulators
        status = isi_get_stream_field(isiIndex, ISI_TIP_LINEAR_VELOCITY, &streamData);
        if (status != ISI_SUCCESS) {
            CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_TIP_LINEAR_VELOCITY, manipulator  \""
                                    << ManipulatorIndexToString(index) << "\", status: "
                                    << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        } else {
            // check size of data, 1 vector
            if (streamData.count != 3) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_TIP_LINEAR_VELOCITY, manipulator \""
                                        << ManipulatorIndexToString(index) << "\", expected 3, received "
                                        << streamData.count << std::endl;
                arm->m_measured_cv.Valid() = false;
            } else {
                ISI_VECTOR * isiTransform = reinterpret_cast<ISI_VECTOR *>(streamData.data);
                vctDouble3 & vel = arm->m_measured_cv.VelocityLinear();
                vel[0] = isiTransform->x;
                vel[1] = isiTransform->y;
                vel[2] = isiTransform->z;
                arm->m_measured_cv.Valid() = true;
            }
        }

        // tip angular velocity for all manipulators
        status = isi_get_stream_field(isiIndex, ISI_TIP_ANGULAR_VELOCITY, &streamData);
        if (status != ISI_SUCCESS) {
            CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_TIP_ANGULAR_VELOCITY, manipulator  \""
                                    << ManipulatorIndexToString(index) << "\", status: "
                                    << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        } else {
            // check size of data, 1 vector
            if (streamData.count != 3) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_TIP_ANGULAR_VELOCITY, manipulator \""
                                        << ManipulatorIndexToString(index) << "\", expected 3, received "
                                        << streamData.count << std::endl;
                arm->m_measured_cv.Valid() = false;
            } else {
                ISI_VECTOR * isiTransform = reinterpret_cast<ISI_VECTOR *>(streamData.data);
                vctDouble3 & vel = arm->m_measured_cv.VelocityAngular();
                vel[0] = isiTransform->x;
                vel[1] = isiTransform->y;
                vel[2] = isiTransform->z;
                arm->m_measured_cv.Valid() = true;
            }
        }

        // rcm transform for the patient side and endoscopic manipulators
        if ((index >= PSM1) && (index <= ECM1)) {
            _psm = reinterpret_cast<PSMArmData *>(arm);

            status = isi_get_stream_field(isiIndex, ISI_RCM_TRANSFORM, &streamData);
            if (status != ISI_SUCCESS) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_RCM_TRANSFORM, manipulator  \""
                                        << ManipulatorIndexToString(index) << "\", status: "
                                        << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
            } else {
                // check size of data, 4 vectors
                if (streamData.count != 12) {
                    CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_RCM_TRANSFORM, manipulator \""
                                            << ManipulatorIndexToString(index) << "\", expected 12, received "
                                            << streamData.count << std::endl;
                    _psm->PositionCartesianRCM.Valid() = false;
                } else {
                    ISI_TRANSFORM * isiTransform = reinterpret_cast<ISI_TRANSFORM *>(streamData.data);
                    mtsIntuitiveDaVinciUtilities::FrameFromISI(*isiTransform,
                                                               _psm->PositionCartesianRCM.Position());
                    _psm->PositionCartesianRCM.Valid() = true;
                }
            }

            // mount transform for the patient side and endoscopic manipulators
            status = isi_get_stream_field(isiIndex, ISI_MOUNT_TRANSFORM, &streamData);
            if (status != ISI_SUCCESS) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_MOUNT_TRANSFORM, manipulator  \""
                                        << ManipulatorIndexToString(index) << "\", status: "
                                        << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
            } else {
                // check size of data, 4 vectors
                if (streamData.count != 12) {
                    CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_MOUNT_TRANSFORM, manipulator \""
                                            << ManipulatorIndexToString(index) << "\", expected 12, received "
                                            << streamData.count << std::endl;
                    _psm->PositionCartesianSetup.Valid() = false;
                } else {
                    ISI_TRANSFORM * isiTransform = reinterpret_cast<ISI_TRANSFORM *>(streamData.data);
                    mtsIntuitiveDaVinciUtilities::FrameFromISI(*isiTransform,
                                                               _psm->PositionCartesianSetup.Position());
                    _psm->PositionCartesianSetup.Valid() = true;
                }
            }

            // setup joint values for the patient side and endoscopic manipulators
            status = isi_get_stream_field(isiIndex, ISI_SUJ_JOINT_VALUES, &streamData);
            if (status != ISI_SUCCESS) {
                CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: get stream field failed for ISI_SUJ_JOINT_VALUES, manipulator  \""
                                        << ManipulatorIndexToString(index) << "\", status: "
                                        << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
            } else {
                numberOfJoints = daVinci::NUM_SUJ_JOINTS;
                // check size of data
                if (streamData.count != numberOfJoints) {
                    CMN_LOG_CLASS_RUN_ERROR << "StreamCallback: received wrong number of elements for ISI_SUJ_JOINT_VALUES, manipulator \""
                                            << ManipulatorIndexToString(index) << "\", expected "
                                            << numberOfJoints << ", received "
                                            << streamData.count << std::endl;
                    _psm->StateSUJ.Valid() = false;
                } else {
                    // save the joint values
                    vctDynamicConstVectorRef<float> jointPos;
                    jointPos.SetRef(numberOfJoints, streamData.data);
                    _psm->StateSUJ.Position().Assign(jointPos);
                    _psm->StateSUJ.Valid() = true;
                }
            }
        }
        // advance state table for this arm
        arm->StateTable->Advance();
        // trigger event to indicate new data stream available
        arm->DataUpdated();
    }

}


bool mtsIntuitiveDaVinci::ConfigureEvents(void)
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "ConfigureEvents: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return false;
    }
    // API calls
    ISI_STATUS status;
    status = isi_subscribe_all_events();
    if (status != ISI_SUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureEvents: subscribe all events failed, status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        return false;
    }

    // register callback
    status = isi_set_event_callback(mtsIntuitiveDaVinciUtilities::EventCallback, this);
    if (status != ISI_SUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureEvent: failed to set event callback, status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        return false;
    }

    return true;
}


void mtsIntuitiveDaVinci::EventCallback(ManipulatorIndexType manipulatorIndex, int eventId, int* eventArgs)
{
    // trigger the void event using ISI name
    std::vector<std::string> v;
    // manipulator source of the event
    v.push_back(ManipulatorIndexToString(manipulatorIndex));
    // event name
    v.push_back(Events.EventNames[eventId]);
    // event arguments
    char temp[50];
    sprintf(temp, "%d", eventArgs[0]);
    v.push_back(std::string(temp));
    sprintf(temp, "%d", eventArgs[1]);
    v.push_back(std::string(temp));
    sprintf(temp, "%d", eventArgs[2]);
    v.push_back(std::string(temp));
    sprintf(temp, "%d", eventArgs[3]);
    v.push_back(std::string(temp));
    (Events.WriteFunctions[eventId])->Execute(v);

    prmEventButton buttonPayload;
    buttonPayload.SetValid(true);
    buttonPayload.SetTimestamp(mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime());

    switch (eventId) {
        // console events
    case ISI_API_MASTER_CLUTCH_ON:
        buttonPayload.SetType(prmEventButton::PRESSED);
        this->Console.Clutch(buttonPayload);
        this->Console.Clutched = true;
        // reset clutch angles on masters
        this->MTMArms[0]->ClutchRestAngleNeedsUpdate = true;
        this->MTMArms[1]->ClutchRestAngleNeedsUpdate = true;
        switch (manipulatorIndex) {
        case MTML1:
        case MTMR1:
        case CONSOLE1:
            this->Consoles[0]->Clutch(buttonPayload);
            this->Consoles[0]->Clutched = true;
            break;
        case MTML2:
        case MTMR2:
        case CONSOLE2:
            this->Consoles[1]->Clutch(buttonPayload);
            this->Consoles[1]->Clutched = true;
            break;
        default:
            CMN_LOG_CLASS_RUN_WARNING << "EventCallBack: manipulator index not supported for event ISI_API_MASTER_CLUTCH_ON" << std::endl;
        }
        break;
    case ISI_API_MASTER_CLUTCH_OFF:
        buttonPayload.SetType(prmEventButton::RELEASED);
        this->Console.Clutch(buttonPayload);
        this->Console.Clutched = false;
        // exit MaM if needed
        if (this->Console.MastersAsMiced) {
            buttonPayload.SetType(prmEventButton::RELEASED);
            this->Console.MastersAsMice(buttonPayload);
            this->Console.MastersAsMiced = false;
        }
        switch (manipulatorIndex) {
        case MTML1:
        case MTMR1:
        case CONSOLE1:
            this->Consoles[0]->Clutch(buttonPayload);
            this->Consoles[0]->Clutched = true;
            break;
        case MTML2:
        case MTMR2:
        case CONSOLE2:
            this->Consoles[1]->Clutch(buttonPayload);
            this->Consoles[1]->Clutched = true;
            break;
        default:
            CMN_LOG_CLASS_RUN_WARNING << "EventCallBack: manipulator index not supported for event ISI_API_MASTER_CLUTCH_OFF" << std::endl;
        }
        break;
    case ISI_API_CAMERA_CONTROL_ON:
        buttonPayload.SetType(prmEventButton::PRESSED);
        this->Console.Camera(buttonPayload);
        break;
    case ISI_API_CAMERA_CONTROL_OFF:
        buttonPayload.SetType(prmEventButton::RELEASED);
        this->Console.Camera(buttonPayload);
        break;
    case ISI_API_HEAD_IN:
        this->Console.HeadIn();
        buttonPayload.SetType(prmEventButton::PRESSED);
        this->Console.OperatorPresent(buttonPayload);
        break;
    case ISI_API_HEAD_OUT:
        this->Console.HeadOut();
        buttonPayload.SetType(prmEventButton::RELEASED);
        this->Console.OperatorPresent(buttonPayload);
        break;
    case ISI_API_ARM_SWAP:
        this->Console.ClutchQuickTap();
        break;
    case ISI_API_VIDEO_SWAP:
        this->Console.CameraQuickTap();
        break;
    case ISI_API_FOLLOWING_ON:
        buttonPayload.SetType(prmEventButton::PRESSED);
        this->Console.FollowMode(buttonPayload);
        this->Arms[manipulatorIndex]->FollowMode(buttonPayload);
        break;
    case ISI_API_FOLLOWING_OFF:
        buttonPayload.SetType(prmEventButton::RELEASED);
        this->Console.FollowMode(buttonPayload);
        this->Arms[manipulatorIndex]->FollowMode(buttonPayload);
        break;
    case ISI_API_MTM_GRIP_A_UP:
    case ISI_API_MTM_GRIP_B_UP:
        buttonPayload.SetType(prmEventButton::RELEASED);
        this->MTMArms[manipulatorIndex - MTML1]->Clutch(buttonPayload);
        break;
    case ISI_API_MTM_GRIP_A_DOWN:
    case ISI_API_MTM_GRIP_B_DOWN:
        buttonPayload.SetType(prmEventButton::PRESSED);
        this->MTMArms[manipulatorIndex - MTML1]->Clutch(buttonPayload);
        break;

    default:
        CMN_LOG_CLASS_RUN_WARNING << "EventCallback: event type not handled: " << eventId << std::endl;
        break;
    }
}


bool mtsIntuitiveDaVinci::StartStream(void)
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "StartStream: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return false;
    }
    // API calls
    ISI_STATUS status;
    status = isi_start_stream(this->RateInHz);
    if (status != ISI_SUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "StartStream: start stream failed, status: "
                                 << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
        return false;
    }
    return true;
}


void mtsIntuitiveDaVinci::StopStream(void)
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "StopStream: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return;
    }
    // API calls
    isi_stop_stream();
}


void mtsIntuitiveDaVinci::LogSystemConfiguration(cmnLogLevel logLevel) const
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "LogSystemConfiguration: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return;
    }
    // API call
    ISI_STATUS status;
    ISI_SYSTEM_CONFIG configuration;
    status = isi_get_system_config(&configuration);
    // log results
    if (status == ISI_SUCCESS) {
        CMN_LOG_CLASS(logLevel) << "System configuration for \"" << this->GetName() << "\":" << std::endl
                                << mtsIntuitiveDaVinciUtilities::SystemConfigurationToString(configuration) << std::endl;
    } else {
        CMN_LOG_CLASS_RUN_ERROR << "LogSystemConfiguration: failed to obtain system configuration for \""
                                << this->GetName() << "\", status: "
                                << mtsIntuitiveDaVinciUtilities::StatusToString(status) << std::endl;
    }
}


void mtsIntuitiveDaVinci::LogManipulatorConfiguration(ManipulatorIndexType index,
                                                      cmnLogLevel) const
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "LogManipulatorConfiguration: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return;
    }
    // check the manipulator index
    if ((index < MTML1) || (index > ECM1)) {
        CMN_LOG_CLASS_INIT_ERROR << "LogManipulatorConfiguration: called with incorrect manipulator index" << std::endl;
        return;
    }
}


void mtsIntuitiveDaVinci::LogToolConfiguration(ManipulatorIndexType index,
                                               cmnLogLevel) const
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "LogToolConfiguration: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return;
    }
    // check the manipulator index
    if ((index < PSM1) || (index > PSM3)) {
        CMN_LOG_CLASS_INIT_ERROR << "LogToolConfiguration: called with incorrect manipulator index" << std::endl;
        return;
    }
}


void mtsIntuitiveDaVinci::LogManipulatorsAndToolsConfiguration(cmnLogLevel logLevel) const
{
    // check if system is connected
    if (!this->Connected) {
        CMN_LOG_CLASS_RUN_ERROR << "LogManipulatorsConfiguration: \"" << this->GetName()
                                << "\" not connected." << std::endl;
        return;
    }
    // log for each manipulator
    ManipulatorIndexType manipulatorIndex;
    for (manipulatorIndex = MTML1;
         manipulatorIndex <= ECM1;
         manipulatorIndex = static_cast<ManipulatorIndexType>(manipulatorIndex + 1)) {
        this->LogManipulatorConfiguration(manipulatorIndex, logLevel);
        if ((manipulatorIndex == PSM1)
            || (manipulatorIndex == PSM2)
            || (manipulatorIndex == PSM3)) {
            this->LogToolConfiguration(manipulatorIndex, logLevel);
        }
    }
}


const std::string & mtsIntuitiveDaVinci::ManipulatorIndexToString(ManipulatorIndexType manipulatorIndex)
{
    static const std::string manipulatorStrings[]
        = {"MTML1",
           "MTMR1",
           "MTML2",
           "MTMR2",
           "PSM1",
           "PSM2",
           "PSM3",
           "ECM1",
           "Console1",
           "Console2",
           "Core"};
    static const std::string errorString("InvalidIndex");
    if (manipulatorIndex < NUMBER_MANIPULATORS) {
        return manipulatorStrings[manipulatorIndex];
    }
    return errorString;
}


size_t mtsIntuitiveDaVinci::GetNumberOfJoints(ManipulatorIndexType manipulatorIndex)
{
    ManipulatorType manipulatorType = mtsIntuitiveDaVinci::GetManipulatorType(manipulatorIndex);
    size_t numJoints = 0;
    switch (manipulatorType) {
    case MTM_TYPE:
        numJoints = ISI::NUM_MTM_JOINTS;
        break;
    case PSM_TYPE:
        numJoints = daVinci::NUM_PSM_JOINTS;
        break;
    case ECM_TYPE:
        numJoints = daVinci::NUM_ECM_JOINTS;
        break;
    default:
        CMN_LOG_RUN_ERROR << " Class mtsIntuitiveDaVinci: GetNumberOfJoints: invalid manipulator type for manipulator index " << manipulatorIndex << std::endl;
    }
    return numJoints;
}

size_t mtsIntuitiveDaVinci::GetNumberOfSetupJoints(ManipulatorIndexType manipulatorIndex)
{
    ManipulatorType manipulatorType = mtsIntuitiveDaVinci::GetManipulatorType(manipulatorIndex);
    size_t numSetupJoints = -1;
    switch (manipulatorType) {
    case MTM_TYPE:
        numSetupJoints = 0;
        break;
    case ECM_TYPE:
    case PSM_TYPE:
        numSetupJoints = daVinci::NUM_SUJ_JOINTS;
        break;
    default:
        CMN_LOG_INIT_ERROR << "Class mtsIntuitiveDaVinci: GetNumberOfSetupJoints: invalid manipulator type: " << manipulatorIndex << "  " << std::endl;
        break;
    }
    return numSetupJoints;
}

mtsIntuitiveDaVinci::ManipulatorType mtsIntuitiveDaVinci::GetManipulatorType(ManipulatorIndexType manipulatorIndex)
{
    switch (manipulatorIndex) {
    case MTML1:
    case MTMR1:
    case MTML2:
    case MTMR2:
        return MTM_TYPE;
    case PSM1:
    case PSM2:
    case PSM3:
        return PSM_TYPE;
    case ECM1:
        return ECM_TYPE;
    case CONSOLE1:
    case CONSOLE2:
        return CONSOLE_TYPE;
    default:
        return static_cast<mtsIntuitiveDaVinci::ManipulatorType> (-1);
    }
}


void mtsIntuitiveDaVinci::SetupArmsInterfaces(void)
{
    CMN_LOG_CLASS_INIT_DEBUG << "SetupArmInterfaces: adding interfaces and state tables" << std::endl;
    ManipulatorIndexType manipulatorIndex;
    ArmData * arm;
    MTMArmData * _mtm;
    PSMArmData * _psm;

    size_t numberOfJoints;
    std::string manipulatorName;
    // create data for all arms
    for (manipulatorIndex = MTML1;
         manipulatorIndex <= ECM1;
         manipulatorIndex = static_cast<ManipulatorIndexType>(manipulatorIndex + 1)) {
        manipulatorName = mtsIntuitiveDaVinci::ManipulatorIndexToString(manipulatorIndex);
        CMN_LOG_CLASS_INIT_DEBUG << "SetupMTMsInterfaces: setting up arm \""
                                 << manipulatorName << "\"" << std::endl;
        // create the structure based on arm type
        if (IsMTM(manipulatorIndex)) {
            _mtm = new MTMArmData;
            arm = _mtm;
        } else if (IsPSM(manipulatorIndex) || IsECM(manipulatorIndex)) {
            _psm= new PSMArmData;
            arm = _psm;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "SetupArmInterfaces: invalid manipulator index" << std::endl;
        }
        CMN_ASSERT(arm);
        Arms[manipulatorIndex - MTML1] = arm;
        // add a state table using the size of the default state table
        arm->StateTable = new mtsStateTable(this->StateTable.GetHistoryLength(), manipulatorName);
        CMN_ASSERT(arm->StateTable);
        this->AddStateTable(arm->StateTable, true);
        arm->StateTable->SetAutomaticAdvance(false);
        // add configuration state table
        arm->ConfigurationStateTable = new mtsStateTable(10, manipulatorName + "Configuration");
        CMN_ASSERT(arm->ConfigurationStateTable);
        this->AddStateTable(arm->ConfigurationStateTable, true);
        arm->ConfigurationStateTable->SetAutomaticAdvance(false);
        // add a provided interface
        arm->ProvidedInterface = this->AddInterfaceProvided(manipulatorName);
        CMN_ASSERT(arm->ProvidedInterface);
        // add default message events
        arm->ProvidedInterface->AddMessageEvents();
        // add a void event to indicate new stream of data has been captured
        arm->ProvidedInterface->AddEventVoid(arm->DataUpdated, "DataUpdated");
        // resize containers for joint information
        numberOfJoints = mtsIntuitiveDaVinci::GetNumberOfJoints(manipulatorIndex);
        arm->m_measured_js.Name().resize(numberOfJoints);

        if (IsMTM(manipulatorIndex)) {
            // kinematic
            _mtm->m_configuration_js.Name().resize(numberOfJoints - 1);
            _mtm->m_configuration_js.Type().resize(numberOfJoints - 1);
            _mtm->m_configuration_js.Name().at(0) = "outer_yaw";
            _mtm->m_configuration_js.Name().at(1) = "shoulder_pitch";
            _mtm->m_configuration_js.Name().at(2) = "elbow_pitch";
            _mtm->m_configuration_js.Name().at(3) = "wrist_platform";
            _mtm->m_configuration_js.Name().at(4) = "wrist_pitch";
            _mtm->m_configuration_js.Name().at(5) = "wrist_yaw";
            _mtm->m_configuration_js.Name().at(6) = "wrist_roll";
            cmnDataCopy(_mtm->m_measured_js.Name(), _mtm->m_configuration_js.Name());
            _mtm->m_measured_js.Position().SetSize(numberOfJoints - 1);
            _mtm->m_measured_js.Velocity().SetSize(numberOfJoints - 1);
            _mtm->m_measured_js.Effort().SetSize(numberOfJoints - 1);
            // gripper
            _mtm->m_gripper_configuration_js.Name().resize(1);
            _mtm->m_gripper_configuration_js.Type().SetSize(1);
            _mtm->m_gripper_configuration_js.Name().at(0) = "finger_grips";
            _mtm->m_gripper_configuration_js.Type().at(0) = PRM_JOINT_REVOLUTE;
            _mtm->ConfigurationStateTable->AddData(_mtm->m_gripper_configuration_js,
                                                   "m_gripper_configuration_js");
            _mtm->ProvidedInterface->AddCommandReadState(*(_mtm->ConfigurationStateTable),
                                                         _mtm->m_gripper_configuration_js,
                                                         "gripper/configuration_js");
            cmnDataCopy(_mtm->m_gripper_measured_js.Name(), _mtm->m_gripper_configuration_js.Name());
            _mtm->m_gripper_measured_js.Position().SetSize(1);
            _mtm->m_gripper_measured_js.Velocity().SetSize(1);
            _mtm->m_gripper_measured_js.Effort().SetSize(0); // MTM doesn't report effort on gripper
            _mtm->StateTable->AddData(_mtm->m_gripper_measured_js, "m_gripper_measured_js");
            _mtm->ProvidedInterface->AddCommandReadState(*(_mtm->StateTable),
                                                         _mtm->m_gripper_measured_js,
                                                         "gripper/measured_js");
        } else if (IsPSM(manipulatorIndex)) {
            // kinematic
            _psm->m_configuration_js.Name().resize(numberOfJoints - 1);
            _psm->m_configuration_js.Type().SetSize(numberOfJoints - 1);
            _psm->m_configuration_js.Name().at(0) = "outer_yaw";
            _psm->m_configuration_js.Name().at(1) = "outer_pitch";
            _psm->m_configuration_js.Name().at(2) = "outer_insertion";
            _psm->m_configuration_js.Name().at(3) = "outer_roll";
            _psm->m_configuration_js.Name().at(4) = "outer_wrist_pitch";
            _psm->m_configuration_js.Name().at(5) = "outer_wrist_yaw";
            cmnDataCopy(_psm->m_measured_js.Name(), _psm->m_configuration_js.Name());
            _psm->m_measured_js.Position().SetSize(numberOfJoints - 1);
            _psm->m_measured_js.Velocity().SetSize(numberOfJoints - 1);
            _psm->m_measured_js.Effort().SetSize(numberOfJoints - 1);
            // jaw
            _psm->m_jaw_configuration_js.Name().resize(1);
            _psm->m_jaw_configuration_js.Type().SetSize(1);
            _psm->m_jaw_configuration_js.Name().at(0) = "jaw";
            _psm->m_jaw_configuration_js.Type().at(0) = PRM_JOINT_REVOLUTE;
            _psm->ConfigurationStateTable->AddData(_psm->m_jaw_configuration_js,
                                                   "m_jaw_configuration_js");
            _psm->ProvidedInterface->AddCommandReadState(*(_psm->ConfigurationStateTable),
                                                         _psm->m_jaw_configuration_js,
                                                         "jaw/configuration_js");
            cmnDataCopy(_psm->m_jaw_measured_js.Name(), _psm->m_jaw_configuration_js.Name());
            _psm->m_jaw_measured_js.Position().SetSize(1);
            _psm->m_jaw_measured_js.Velocity().SetSize(1);
            _psm->m_jaw_measured_js.Effort().SetSize(1);
            _psm->StateTable->AddData(_psm->m_jaw_measured_js, "m_jaw_measured_js");
            _psm->ProvidedInterface->AddCommandReadState(*(_psm->StateTable),
                                                         _psm->m_jaw_measured_js,
                                                         "jaw/measured_js");
        } else if (IsECM(manipulatorIndex)) {
            // kinematic only
            arm->m_configuration_js.Name().resize(numberOfJoints);
            arm->m_configuration_js.Type().SetSize(numberOfJoints);
            arm->m_configuration_js.Name().at(0) = "outer_yaw";
            arm->m_configuration_js.Name().at(1) = "outer_pitch";
            arm->m_configuration_js.Name().at(2) = "insertion";
            arm->m_configuration_js.Name().at(3) = "outer_roll";
            cmnDataCopy(arm->m_measured_js.Name(), arm->m_configuration_js.Name());
            arm->m_measured_js.Position().SetSize(numberOfJoints);
            arm->m_measured_js.Velocity().SetSize(numberOfJoints);
            arm->m_measured_js.Effort().SetSize(numberOfJoints);
        }

        // joint type
        arm->m_configuration_js.Type().SetAll(PRM_JOINT_REVOLUTE);
        if (IsPSM(manipulatorIndex) || IsECM(manipulatorIndex)) {
            arm->m_configuration_js.Type().at(2) = PRM_JOINT_PRISMATIC;
        }

        // add to state table and provided interface
        arm->StateTable->AddData(arm->DeviceTimestamp, "DeviceTimestamp");
        arm->ProvidedInterface->AddCommandReadState(*(arm->StateTable),
                                                    arm->DeviceTimestamp, "GetDeviceTimestamp");
        arm->m_measured_cp.SetMovingFrame(mtsIntuitiveDaVinci::ManipulatorIndexToString(manipulatorIndex));
        arm->m_measured_cv.SetMovingFrame(mtsIntuitiveDaVinci::ManipulatorIndexToString(manipulatorIndex));
        switch (manipulatorIndex) {
        case PSM1:
        case PSM2:
        case PSM3:
            arm->m_measured_cp.SetReferenceFrame("ECM1");
            arm->m_measured_cv.SetReferenceFrame("ECM1");
            break;
        case ECM1:
            arm->m_measured_cp.SetReferenceFrame("Cart1");
            arm->m_measured_cv.SetReferenceFrame("Cart1");
            break;
        case MTML1:
        case MTMR1:
            arm->m_measured_cp.SetReferenceFrame("CONSOLE1");
            arm->m_measured_cv.SetReferenceFrame("CONSOLE1");
            break;
        case MTML2:
        case MTMR2:
            arm->m_measured_cp.SetReferenceFrame("CONSOLE2");
            arm->m_measured_cv.SetReferenceFrame("CONSOLE2");
            break;
        default:
            break;
        }
        arm->StateTable->AddData(arm->m_measured_cp, "m_measured_cp");
        arm->ProvidedInterface->AddCommandReadState(*(arm->StateTable),
                                                    arm->m_measured_cp, "measured_cp");
        arm->StateTable->AddData(arm->m_measured_cv, "m_measured_cv");
        arm->ProvidedInterface->AddCommandReadState(*(arm->StateTable),
                                                    arm->m_measured_cv, "body/measured_cv");
        arm->ConfigurationStateTable->AddData(arm->m_configuration_js, "m_configuration_js");
        arm->ConfigurationStateTable->Advance(); // to make sure latest values are "published"
        arm->ProvidedInterface->AddCommandReadState(*(arm->ConfigurationStateTable),
                                                    arm->m_configuration_js, "configuration_js");
        arm->StateTable->AddData(arm->m_measured_js, "m_measured_js");
        arm->ProvidedInterface->AddCommandReadState(*(arm->StateTable),
                                                    arm->m_measured_js, "measured_js");
        arm->ProvidedInterface->AddCommandReadState(*(arm->StateTable),
                                                    arm->StateTable->PeriodStats,
                                                    "period_statistics");

        // follow mode button interface and event
        arm->FollowModeProvidedInterface = this->AddInterfaceProvided(manipulatorName + "/follow_mode");
        CMN_ASSERT(arm->FollowModeProvidedInterface);
        arm->FollowModeProvidedInterface->AddEventWrite(arm->FollowMode, "Button", prmEventButton());
    }
}


void mtsIntuitiveDaVinci::SetupMTMsInterfaces(void)
{
    CMN_LOG_CLASS_INIT_DEBUG << "SetupMTMsInterfaces: adding master specific events and commands" << std::endl;
    ManipulatorIndexType manipulatorIndex;
    MTMArmData * _mtm;
    std::string manipulatorName;
    for (manipulatorIndex = MTML1;
         manipulatorIndex <= MTMR2;
         manipulatorIndex = static_cast<ManipulatorIndexType>(manipulatorIndex + 1)) {
        // retrieve pointer to arm from Arms array
        _mtm = static_cast<MTMArmData *>(this->Arms[manipulatorIndex]);
        CMN_ASSERT(_mtm);
        MTMArms[manipulatorIndex - MTML1] = _mtm;
        // events and commands specific to master arms
        manipulatorName = mtsIntuitiveDaVinci::ManipulatorIndexToString(manipulatorIndex);
        // select button interface and event
        _mtm->SelectProvidedInterface = this->AddInterfaceProvided(manipulatorName + "/select");
        CMN_ASSERT(_mtm->SelectProvidedInterface);
        _mtm->SelectProvidedInterface->AddEventWrite(_mtm->Select, "Button", prmEventButton());
        // clutch button interface and event
        _mtm->ClutchProvidedInterface = this->AddInterfaceProvided(manipulatorName + "/clutch");
        CMN_ASSERT(_mtm->ClutchProvidedInterface);
        _mtm->ClutchProvidedInterface->AddEventWrite(_mtm->Clutch, "Button", prmEventButton());
    }
}


void mtsIntuitiveDaVinci::SetupPSMsInterfaces(void)
{
    CMN_LOG_CLASS_INIT_DEBUG << "SetupPSMsInterfaces: adding slave specific events and commands" << std::endl;

    ManipulatorIndexType manipulatorIndex;
    PSMArmData * _psm;
    std::string manipulatorName;
    for (manipulatorIndex = PSM1;
         manipulatorIndex <= ECM1;
         manipulatorIndex = static_cast<ManipulatorIndexType>(manipulatorIndex + 1)) {
        // retrieve pointer to arm from Arms array
        _psm = static_cast<PSMArmData *>(this->Arms[manipulatorIndex]);
        CMN_ASSERT(_psm);
        PSMArms[manipulatorIndex - PSM1] = _psm;
        manipulatorName = mtsIntuitiveDaVinci::ManipulatorIndexToString(manipulatorIndex);

        // events and commands specific to slave arms
        // RCM
        _psm->PositionCartesianRCM.SetMovingFrame(manipulatorName + "_RCM");
        _psm->PositionCartesianRCM.SetReferenceFrame("ECM");
        _psm->StateTable->AddData(_psm->PositionCartesianRCM, "PositionCartesianRCM");
        _psm->ProvidedInterface->AddCommandReadState(*(_psm->StateTable),
                                                     _psm->PositionCartesianRCM, "SUJ/measured_cp");
        // Setup joints, cartesian
        _psm->PositionCartesianSetup.SetMovingFrame(manipulatorName + "_SUJ");
        _psm->PositionCartesianSetup.SetReferenceFrame("Cart1");
        _psm->StateTable->AddData(_psm->PositionCartesianSetup, "PositionCartesianSetup");
        _psm->ProvidedInterface->AddCommandReadState(*(_psm->StateTable),
                                                     _psm->PositionCartesianSetup, "SUJ/local/measured_cp");
        // Setup joints, joint
        _psm->ConfigurationSUJ.Name().resize(6);
        for (size_t jointIndex = 0; jointIndex < 6; ++jointIndex) {
            std::stringstream ss;
            ss.str() = "j";
            ss << jointIndex;
            _psm->ConfigurationSUJ.Name().at(jointIndex) = ss.str();
        }
        _psm->ConfigurationSUJ.Type().SetSize(6);
        _psm->ConfigurationSUJ.Type().SetAll(PRM_JOINT_REVOLUTE);
        _psm->ConfigurationSUJ.Type().at(0) = PRM_JOINT_PRISMATIC;
        _psm->ConfigurationStateTable->AddData(_psm->ConfigurationSUJ, "ConfigurationSUJ");
        _psm->ProvidedInterface->AddCommandReadState(*(_psm->ConfigurationStateTable),
                                                     _psm->ConfigurationSUJ, "GetConfigurationJointSetup");
        cmnDataCopy(_psm->StateSUJ.Name(), _psm->ConfigurationSUJ.Name());
        _psm->StateSUJ.Position().SetSize(6);
        _psm->StateTable->AddData(_psm->StateSUJ, "StateSUJ");
        _psm->ProvidedInterface->AddCommandReadState(*(_psm->StateTable),
                                                     _psm->StateSUJ, "SUJ/measured_js");
    }
}


void mtsIntuitiveDaVinci::SetupCameraInterfaces(void)
{
    CMN_LOG_CLASS_INIT_DEBUG << "SetupCamerasInterfaces: adding camera specific events and commands" << std::endl;
    ManipulatorIndexType manipulatorIndex;
    PSMArmData * cameraArm;
    std::string manipulatorName;
    for (manipulatorIndex = ECM1;
         manipulatorIndex <= ECM1;
         manipulatorIndex = static_cast<ManipulatorIndexType>(manipulatorIndex + 1)) {
        // retrieve pointer to arm from Arms array
        cameraArm = static_cast<PSMArmData *>(this->Arms[manipulatorIndex]);
        CMN_ASSERT(cameraArm);
        CameraArms[manipulatorIndex - ECM1] = cameraArm;
        // events and commands specific to camera arms
        // this is a placeholder, I am not sure we will ever need it
    }
}


void mtsIntuitiveDaVinci::SetupConsoleInterfaces(void)
{
    Console.ProvidedInterface = this->AddInterfaceProvided("Console");
    CMN_ASSERT(Console.ProvidedInterface);

    Console.ProvidedInterface->AddEventVoid(Console.HeadIn, "console/head_in");
    Console.ProvidedInterface->AddEventVoid(Console.HeadOut, "console/head_out");
    Console.ProvidedInterface->AddEventVoid(Console.ClutchQuickTap, "console/clutch_quick_tap");
    Console.ProvidedInterface->AddEventVoid(Console.CameraQuickTap, "console/camera_quick_tap");

    Console.OperatorPresentProvidedInterface = this->AddInterfaceProvided("console/operator_present");
    CMN_ASSERT(Console.OperatorPresentProvidedInterface);
    Console.OperatorPresentProvidedInterface->AddEventWrite(Console.OperatorPresent, "Button", prmEventButton());

    Console.ClutchProvidedInterface = this->AddInterfaceProvided("console/clutch");
    CMN_ASSERT(Console.ClutchProvidedInterface);
    Console.ClutchProvidedInterface->AddEventWrite(Console.Clutch, "Button", prmEventButton());

    Console.CameraProvidedInterface = this->AddInterfaceProvided("console/camera");
    CMN_ASSERT(Console.CameraProvidedInterface);
    Console.CameraProvidedInterface->AddEventWrite(Console.Camera, "Button", prmEventButton());

    Console.MastersAsMiceProvidedInterface = this->AddInterfaceProvided("console/masters_as_mice");
    CMN_ASSERT(Console.MastersAsMiceProvidedInterface);
    Console.MastersAsMiceProvidedInterface->AddEventWrite(Console.MastersAsMice, "Button", prmEventButton());

    Console.FollowModeProvidedInterface = this->AddInterfaceProvided("console/follow_mode");
    CMN_ASSERT(Console.FollowModeProvidedInterface);
    Console.FollowModeProvidedInterface->AddEventWrite(Console.FollowMode, "Button", prmEventButton());

    ManipulatorIndexType manipulatorIndex;
    ConsoleData* console;
    std::string manipulatorName;
    for (manipulatorIndex = CONSOLE1;
         manipulatorIndex <= CONSOLE2;
         manipulatorIndex = static_cast<ManipulatorIndexType>(manipulatorIndex + 1)) {
        manipulatorName = mtsIntuitiveDaVinci::ManipulatorIndexToString(manipulatorIndex);
        console = new ConsoleData();
        CMN_ASSERT(console);
        Consoles[manipulatorIndex - CONSOLE1] = console;
        console->ProvidedInterface = this->AddInterfaceProvided(manipulatorName);
        CMN_ASSERT(console->ProvidedInterface);

        console->ProvidedInterface->AddEventVoid(console->HeadIn, "console/head_in");
        console->ProvidedInterface->AddEventVoid(console->HeadOut, "console/head_out");
        console->ProvidedInterface->AddEventVoid(console->ClutchQuickTap, "console/clutch_quick_tap");
        console->ProvidedInterface->AddEventVoid(console->CameraQuickTap, "console/camera_quick_tap");
    }
}


void mtsIntuitiveDaVinci::SetupEventInterfaces(void)
{
    Events.ProvidedInterface = this->AddInterfaceProvided("Events");
    CMN_ASSERT(Events.ProvidedInterface);

    Events.WriteFunctions.SetSize(NUM_EVENT_IDS);
    Events.WriteFunctions.SetAll(0);
    Events.EventNames.SetSize(NUM_EVENT_IDS);
    Events.EventNames.SetAll("");

    for (size_t i = 0; i < Events.WriteFunctions.size(); i++) {
        Events.EventNames[i] = isi_get_event_name(static_cast<ISI_EVENT_ID>(i));
        Events.WriteFunctions[i] = new mtsFunctionWrite();
        Events.ProvidedInterface->AddEventWrite(*(Events.WriteFunctions[i]), Events.EventNames[i], std::vector<std::string>());
    }

}


void mtsIntuitiveDaVinci::SetupAllInterfaces(void)
{
    this->SetupEventInterfaces();
    // this must be called first
    this->SetupArmsInterfaces();
    // arm specific extra data
    this->SetupMTMsInterfaces();
    this->SetupPSMsInterfaces();
    this->SetupCameraInterfaces();
    // console interface
    this->SetupConsoleInterfaces();
}
