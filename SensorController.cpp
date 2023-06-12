//****************************************************
// Motorola Solutions Confidential Proprietary
// Copyright 2017 - 2023 Motorola Solutions, Inc.  All Rights Reserved
//
//  File:        SensorController.cpp
//  Author:      Chang How Tan
//  Description
//  This file contains implementation for SensorController class.
//****************************************************
//----------------------------- MACKINAW REVISIONS ---------------------------------
// Date        Name          Description
// --------    ---------     -------------------------------------------------------
// 03/28/18    tfgk46        ASTRO_SUB-7257 Added Sensor emergency type enums, launchVestPiercedEmergency function and handling for SENSOR_EMERGENCY_ACTION case.
// 03/27/18    ngkd64        ASTRO_SUB_7564 Added dsi_read_last_ota_sensor_event_state to check for sensor OTA state.
// 03/19/18    ngkd64        ASTRO_SUB-7095 Added dsi_reb_emer_info_get_call_status to check for emergency state.
// 03/14/18    vmw467        ASTRO_SUB-7255 Added support for vest pierced sensor.
// 03/07/14    ngkd64        ASTRO_SUB-7133 Added handling for GENERIC_SIG_REQ_DENY_BY_SCAN_ON.
// 02/14/18    ngkd64        ASTRO_SUB-6716 Revert B-317098 back to SENSOR_TX_OFF instead of SENSOR_VC_OFF.
// 01/19/18    tfgk46        ASTRO_SUB-4026 Renamed holsterEvent to sensorEvent, holsterTimId to sensorTimId, holsterTimId to sensorTimId,
//                                          holster_event to sensor_event, holster_sequence_id to sensor_sequence_id, pHolsterEvent to pSensorEvent,
//                                          setOldestHolsterData to setOldestSensorData, HOLSTER_EVENT_HANDLE_CONFLICTING_APPS to SENSOR_EVENT_HANDLE_CONFLICTING_APPS,
//                                          vr_env_put_sensor_oldest_holster_event to vr_env_put_sensor_oldest_event, IsSameHolsterEventAndTimId to IsSameSensorEventAndTimId,
//                                          vr_env_put_sensor_oldest_holster_sequence_id to vr_env_put_sensor_oldest_sequence_id.
// 01/03/18    tfgk46        ASTRO_SUB-4444 Added m_weaponFiredTimerHandler, manageWeaponFiredTimer function and SENSOR_WEAPON_FIRED_TIMER_EXPIRED case.
// 12/19/17    tfgk46        ASTRO_SUB-4020 Added support for WEAPON_FIRED and renamed HOLSTER_UNKNOWN to UNKNOWN_STATE, m_sensorData to m_sensors,
//                                          m_holsterData to m_sensorData, m_sensorData to m_sensors, SensorData to Sensors, HolsterData to SensorData.
// 12/18/17    ngkd64        ASTRO_SUB-4353 Change notify parameter to FALSE while launching APP_SCAN_ACTIVE via app2app.
// 12/06/17    ngkd64        ASTRO_SUB-3714 Added handling for SENSOR_TX_INH_OFF.
// 12/06/17    tfgk46        B-317097       Added handling for HOLSTER_EVENT_HANDLE_CONFLICTING_APPS opcode in handleConflictingApps function.
// 11/29/17    tfgk46        B-317099       Added handleConflictingApps function and
//                                          added handling for SENSOR_VC_ON and SENSOR_EXTERNAL_KEYLOADER_DETACHED case,
//                                          replaced svcHEBPutRadioTxing with dsi_reb_get_vc_on_state.
// 11/28/17    ngkd64        B-317098       Replaced handling for SENSOR_TX_OFF with SENSOR_VC_OFF.
// 11/24/17    ngkd64        B-317803       Added handling such that try to turn off APP_SCAN_ACTIVE then initiate signalling sequence.
//                                          Added handling for SENSOR_WAIT_FOR_SCAN_OFF and SENSOR_INIT_SIGNALLING_SEQUENCE.
//                                          Added function isInitSigSequenceAllowed and bool variable m_needToResumeScan.
// 11/20/17    ngkd64        B-317587       Renamed dsi_read_taser_holster_reliable_report_enable and dsi_read_gun_holster_reliable_report_enable
//                                          to dsi_read_taser_state_reliable_reporting_enable and dsi_read_gun_holster_state_reliable_reporting_enable.
// 11/14/17    ngkd64        B-317189       Replace SENSOR_EVENT_UPDATE with SENSOR_SIG_REQ_ACK_STATUS.
// 11/02/17    tfgk46        B-314166       Added setOldestHolsterData function.
// 10/31/17    ngkd64        B-315552       Checked for state changes before storing into a queue.
//                                          Renamed initSignallingData to initSigSequence.
// 10/26/17    tfgk46        B-314824       Added MAX_SENSOR_ALERT_RETRIES, initialised m_sensorEventMsgWaitForAck and m_sensorAlertRetries in constructor,
//                                          updated SENSOR_HOLSTER_EVENT_ACTION, SENSOR_ALERT_UPDATE and SENSOR_EVENT_UPDATE cases.
// 10/12/17    ngkd64        B-314824       Added initSignallingData function.
// 10/09/17    tfgk46        B-314823       Updated SENSOR_HOLSTER_EVENT_ACTION, added SENSOR_TX_OFF case,
//                                          implemented IsSameHolsterEvent class template.
// 10/04/17    ngkd64        B-314821       Added checking in HOLSTER_EVENT_ACTION and added SENSOR_ALERT_UPDATE case.
// 09/27/17    ngkd64        B-314076       Added HOLSTER_EVENT_ACTION case in mainEntry and implemented IsSameHolsterEventAndTimId class template.
// 09/26/17    tfgk46        B-314075       Added static constants HOLSTER_UNKNOWN, WEAPON_DRAWN, WEAPON_HOLSTERED, STUN_GUN_DRAWN,
//                                          STUN_GUN_HOLSTERED to support sensor weapon holster event.
// 08/30/17    tfgk46        B-308361       Updated destructor and SENSOR_STATUS_UNREG case to handle BT turnoff and shutdown.
// 08/24/17    ngkd64        B-304997       Added support for multiple sensors and removed untrackContext when sensor is offline.
//                                          Moved HANDLE_DISPLAY_NOTIFICATION, HANDLE_AUDIBLE_NOTIFICATION, PERIODIC_AUDIBLE_TIMEOUT to NotificationController.
//                                          Removed removeDisplayNotification, manageAudiblePeriodicTimer, removeAudibleNotification.
// 08/16/17    tfgk46        B-308359       Added manageAudiblePeriodicTimer and removeAudibleNotification functions, PERIODIC_AUDIBLE_TIMEOUT a2a case
//                                          and support for Sensor Tone Periodic Timer.
// 08/10/17    tfgk46        B-304998       Implemented removeDisplayNotification function and
//                                          updated SENSOR_CONTEXT_UNREG and HANDLE_DISPLAY_NOTIFICATION a2a cases.
// 08/08/17    ngkd64        B-306186       Implemented display and audible notifications.
//                                          Added sensorLineAttributeTable.
// 08/02/17    ngkd64        B-306185       Added HANDLE_DISPLAY_NOTIFICATION and HANDLE_AUDIBLE_NOTIFICATION a2a case in mainEntry.
//                                          Modified strncpy to reserve space for EOS.
// 07/31/15    tfgk46        B-306183       Implemented new instance for AppSensorContextListener and
//                                          updated cases for SENSOR_CONTEXT_REG, SENSOR_CONTEXT_UNREG
//                                          and added a new case SENSOR_STATUS_UNREG.
// 07/25/17    ngkd64        B-304975       Send a2a msg to AppActionConsolidation to process list of actions in SENSOR_CONTEXT_ACTION case.
//                                          Added m_timName.
// 07/19/17    ngkd64        B-306182       Implemented new instance for AppSensorStatusListener.
// 07/11/17    ngkd64        B-306864       Initial creation.
//********************************************************************************
#include "SensorController.h"
#include "rs_application_id.h"
#include "rs_action_consolidation_dsifs.h"
#include "RsSensorUtils.h"
#include "AppSensorContextListener.h"
#include "AppSensorStatusListener.h"
#include "SensorManagerWrapper.h"
#include "EuaTEDS.h"
#include "ContextRule.h"
#include "AppBgsrvSensorEvent.h"
#include "dsi_com_reb.h"
#include <algorithm>
#include "svc_heb_services.h"
#include "svc_clim_msg_formatter.h"
#include "dsi_com_utilities.h"
#include "GenericSigReqIntf.h"
#include "vr_environment_misc.h"
#include "dsi_com_mode_info_access.h"
#include "svc_tone_services.h"
#include "TedsFactory.h"
#include "AppBackgroundServices.h"
#include "VirtualPartnerAlertController.h"

extern "C"
{
#include "dsi_com_bluetooth_access.h"
}

#define  ALL_SENSOR_FEATURE_ID    0xFFFF
#define  MAX_SENSOR_ALERT_RETRIES 4     // Maximum attempts is 5 (first attempt + 4 retry)
#define  VOICE_CHANNEL_AVAILABLE  0
#define  VOICE_CHANNEL_OCCUPIED   1
#define  TSBK_RETRY_TIMER         1

// Holster and Yardarm sensor states
const UINT8 SensorController::UNKNOWN_STATE      = 0x0;
const UINT8 SensorController::WEAPON_DRAWN       = 0x1;
const UINT8 SensorController::WEAPON_HOLSTERED   = 0x2;
const UINT8 SensorController::STUN_GUN_DRAWN     = 0x3;
const UINT8 SensorController::STUN_GUN_HOLSTERED = 0x4;
const UINT8 SensorController::WEAPON_FIRED       = 0x5;

// Sensor Emergency type ID
const UINT8 SensorController::UNKNOWN_TYPE_ID    = 0x0;
const UINT8 SensorController::VEST_PIERCED       = 0x1;

// Vest Pierced Sensor emergency type Event
const UINT8 SensorController::VEST_PIERCED_UNKNOWN         = 0x0;
const UINT8 SensorController::VEST_PIERCED_UPPER           = 0x1;
const UINT8 SensorController::VEST_PIERCED_LOWER           = 0x2;
const UINT8 SensorController::VEST_PIERCED_UPPER_AND_LOWER = 0x3;


template <class T>
class IsSameTimId
{
public:
    IsSameTimId(UINT16 timId):
    m_timId(timId)
    {
    }

    bool operator()(const T& val)
    {
        return (m_timId == val.timId);
    }

private:
    UINT16 m_timId;
};

template <class T>
class IsSameSensorEventAndTimId
{
public:
    IsSameSensorEventAndTimId(UINT8 sensorEvent, UINT16 sensorTimId):
    m_sensorEvent(sensorEvent),
    m_sensorTimId(sensorTimId)
    {
    }

    bool operator()(const T& val)
    {
        return ((m_sensorEvent == val.sensorEvent) && (m_sensorTimId == val.sensorTimId));
    }
    dsi_action_list_group_t* m_vpAlertActionGroup;

private:
    UINT8 m_sensorEvent;
    UINT16 m_sensorTimId;
};

//--------------------------------------------------------------------------------
//  Function:     SensorController constructor
//
//  Abstract
//      This is constructor for class SensorController
//--------------------------------------------------------------------------------
SensorController::SensorController(void):
m_appSensorStatusListener(NULL),
m_appSensorContextListener(NULL),
m_sensorEventMsgWaitForAck(false),
m_sigReqSrvConflicted(false),
m_needToResumeScan(false),
m_sensorAlertRetries(0),
m_vestPiercedEmergencyActive(false),
m_relaunchVestPierced(false),
m_vcStat(VOICE_CHANNEL_AVAILABLE),
m_tsbkRetryTimerHandler(0)
{
}

//--------------------------------------------------------------------------------
//  Function:     manageSensorTimer
//
//  Abstract
//      This function handles the periodic tone timer operation such as allocating, deallocating and restarting the timer.
//--------------------------------------------------------------------------------
void SensorController::manageSensorTimer(SensorTimerRequests timerRequest)
{
    switch(timerRequest)
    {
        case START_TIMER:
        {
            if (m_tsbkRetryTimerHandler == 0)
            {
                m_tsbkRetryTimerHandler = vr_get_rtc_l_timer();

                if (m_tsbkRetryTimerHandler != 0)
                {
                    vr_set_rtc_l_user_id(m_tsbkRetryTimerHandler, getAppId());
                    vr_set_rtc_l_user_stamp(m_tsbkRetryTimerHandler, AppBgServiceEvent::getTimeStampPerService(AppBgServiceEvent::SENSOR_MON, AppBgsrvSensorEvent::SENSOR_TSBK_RETRY_TIMER));
                }
            }

            if (m_tsbkRetryTimerHandler != 0)
            {
                vr_write_rtc_l_timer(m_tsbkRetryTimerHandler, TSBK_RETRY_TIMER);
            }
            break;
        }
        case FREE_TIMER:
        {
            if (m_tsbkRetryTimerHandler != 0)
            {
                vr_write_rtc_l_timer(m_tsbkRetryTimerHandler, 0);
                vr_free_rtc_l_timer(m_tsbkRetryTimerHandler);
                m_tsbkRetryTimerHandler = 0;
            }
            break;
        }
        default:
        {
            // do nothing
            break;
        }
    }
}

//--------------------------------------------------------------------------------
//  Function:     SensorController destructor
//
//  Abstract
//      This is destructor for class SensorController
//--------------------------------------------------------------------------------
SensorController::~SensorController(void)
{
    AppBgServiceEvent::registerServiceController(AppBgServiceEvent::SENSOR_MON, -1);

    if (m_tsbkRetryTimerHandler != 0)
    {
        // Reset and free timer
        manageSensorTimer(FREE_TIMER);
    }

    for (int i = 0; i < m_sensors.size(); i++)
    {
        if (m_sensors[i].timId > 0)
        {
            // untrack context based on the timId and eventQualifier for all the sensor feature Id
            untrackContext(m_appSensorContextListener, m_sensors[i].timId, ALL_SENSOR_FEATURE_ID, m_sensors[i].eventQualifier);
            
            AppBackgroundServices *bgServiceManager = dynamic_cast<AppBackgroundServices *>(getAppManager());
            bgServiceManager->sendAcRequest(ACTION_EXECUTE, rs_bgsrv_load_clear_display_tone_actions(m_sensors[i].timId), 0, AppBgServiceEvent::SENSOR_MON);
          
        }
    }
    m_sensors.clear();
    m_sensorData.clear();

    delete m_appSensorStatusListener;
    delete m_appSensorContextListener;
    // Reset emergency vest pierced enabled
    dsi_reb_put_emergency_vest_pierced_enabled(FALSE);
}

//--------------------------------------------------------------------------------
//  Function:     Sensors constructor
//
//  Abstract
//      This is constructor for class Sensors
//--------------------------------------------------------------------------------
SensorController::Sensors::Sensors(UINT16 tim_id, char* device_name, UINT8 last_state, UINT8 sequence_id, ContextHMIEvent * event_qualifier):
timId(tim_id),
lastState(last_state),
sequenceId(sequence_id),
eventQualifier(event_qualifier)
{
    strncpy(deviceName, device_name, MAX_DISPLAY_TEXT_SZ - 1);
}

//--------------------------------------------------------------------------------
//  Function:     mainEntry
//
//  Abstract
//      This is main entry for sensor monitor controller.
//--------------------------------------------------------------------------------
bool SensorController::mainEntry(AppEvent* pEvent)
{
    bool controllerDone = false;
    AppBgServiceEvent* tmpEvent = (AppBgServiceEvent*)pEvent;

    if (tmpEvent->getServiceType() == AppBgServiceEvent::SENSOR_MON)
    {
        AppBgsrvSensorEvent * sensorInput = (AppBgsrvSensorEvent*) tmpEvent;
        const EuaTEDS* euaTEDS = NULL;

        switch(sensorInput->getInputType())
        {
            case AppBgsrvSensorEvent::SENSOR_STATUS_REG:
            {
                if (m_appSensorStatusListener == NULL)
                {
                    m_appSensorStatusListener = new AppSensorStatusListener();
                }
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_STATUS_UNREG:
            {
                controllerDone = true;

                break;
            }
            case AppBgsrvSensorEvent::SENSOR_CONTEXT_REG:
            {
                if (m_appSensorContextListener == NULL)
                {
                    m_appSensorContextListener = new AppSensorContextListener();
                }

                uint16_t timId = sensorInput->getTimId();

                std::vector<Sensors>::iterator it = std::find_if(m_sensors.begin(), m_sensors.end(), IsSameTimId<Sensors>(timId));

                if (it == m_sensors.end())
                {
                    // Pass timID to SensorManagerWrapper to obtain EuaTEDS
                    euaTEDS = requestSensorEUATeds(timId);

                    if ((euaTEDS != NULL) && (strncmp((char*)(euaTEDS->getContextClassifier()), "Weapon", strlen("Weapon")) == 0))
                    {
                        uint16_t numCtxRules = euaTEDS->getContextRuleArraySize();
                        uint16_t chId = 0;
                        uint16_t totalChId = 0;
                        const ContextRule* pCtxRule = NULL;

                        for (uint16_t index = 0; index < numCtxRules; index++)
                        {
                            const char * pEventStrPattern = 0;
                            pCtxRule = euaTEDS->getContextRule(index);

                            if(pCtxRule)
                            {
                                pEventStrPattern = (char*)pCtxRule->getCtxEvent();
                            }

                            if (pEventStrPattern &&
                                (
                                 (!strncmp(pEventStrPattern, "VEST_PIERCED", strlen("VEST_PIERCED")))
                                 ||
                                 ((dsi_read_acknowledged_weapon_fired_event_reporting_enable() == TRUE) &&
                                  !(strncmp(pEventStrPattern, "YARDARM", strlen("YARDARM"))))
                                 ||
                                 ((dsi_read_gun_holster_state_reliable_reporting_enable() == TRUE) &&
                                  !strncmp(pEventStrPattern, "HOLSTER", strlen("HOLSTER")))
                                 ||
                                 ((dsi_read_taser_state_reliable_reporting_enable() == TRUE) &&
                                  !strncmp(pEventStrPattern, "TASER", strlen("TASER")))
                                 ||
                                 ((dsi_read_sensor_context_event_notification() == TRUE) &&
                                  !strncmp(pEventStrPattern, "BATTERY_STATE", strlen("BATTERY_STATE")))
                                )
                               )
                            {
                                chId = pCtxRule->getChannelId();

                                if (chId > 0)
                                {
                                    totalChId |= (1 << (chId - 1));
                                }
                            }
                        }

                        if (totalChId > 0)
                        {
                            // Create event qualifier object with ownership transferred
                            trackContext(m_appSensorContextListener, timId, totalChId, new ContextHMIEvent(timId), getRearmTriggerRate(timId));

                            // Save new sensor data entry: use new ContextHMIEvent instance as the previous object ownership was transferred
                            m_sensors.push_back(Sensors(timId, sensorInput->getTimName(), UNKNOWN_STATE, 0, new ContextHMIEvent(timId)));
                        }
                    }
                }
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_CONTEXT_UNREG:
            {
                uint16_t timId = sensorInput->getTimId();

                std::vector<Sensors>::iterator it = std::find_if(m_sensors.begin(), m_sensors.end(), IsSameTimId<Sensors>(timId));
                if (it != m_sensors.end())
                {
                    if (timId > 0)
                    {
                        // untrack context based on the and timId eventQualifier for all the sensor feature Id
                        untrackContext(m_appSensorContextListener, timId, ALL_SENSOR_FEATURE_ID, it->eventQualifier);
                    }
                    m_sensors.erase(it);

                    action_consolidation_request_msg_t action_consolidation_msg;
                    action_consolidation_msg.command = ACTION_EXECUTE;
                    action_consolidation_msg.actionIndex = 0;
                    action_consolidation_msg.action_list_group = rs_bgsrv_load_clear_display_tone_actions(timId);
                    app_action_consolidation_execution_req(getAppId(), &action_consolidation_msg, TRUE);
                }
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_CONTEXT_ACTION:
            {
                std::vector<Sensors>::iterator it = std::find_if(m_sensors.begin(), m_sensors.end(), IsSameTimId<Sensors>(sensorInput->getSensorContext()->tim_id));
                if (it != m_sensors.end())
                {
                    action_consolidation_request_msg_t action_consolidation_msg;
                    action_consolidation_msg.command = ACTION_EXECUTE;
                    action_consolidation_msg.actionIndex = 0;
                    action_consolidation_msg.action_list_group = rs_bgsrv_load_sensor_action_list(sensorInput->getSensorContext(), it->deviceName);
                    app_action_consolidation_execution_req(getAppId(), &action_consolidation_msg, TRUE);
                }
                break;
            }
            // Handle sensor event TSBK action
            case AppBgsrvSensorEvent::SENSOR_EVENT_ACTION:
            {
                sensor_event_app2app_msg_t* pSensorEvent = sensorInput->getSensorEvent();

                if ((pSensorEvent != NULL) && (pSensorEvent->sensor_event != UNKNOWN_STATE))
                {
                    std::list<SensorData>::iterator itSensorData = std::find_if(m_sensorData.begin(), m_sensorData.end(), IsSameSensorEventAndTimId<SensorData>(pSensorEvent->sensor_event, (UINT16)(pSensorEvent->requester_id)));
                    std::vector<Sensors>::iterator itSensors = std::find_if(m_sensors.begin(), m_sensors.end(), IsSameTimId<Sensors>((UINT16)(pSensorEvent->requester_id)));

                    if (itSensorData != m_sensorData.end())
                    {
                        // Remove it then enqueue latest data
                        m_sensorData.erase(itSensorData);
                    }

                    if (itSensors != m_sensors.end())
                    {
                        // If initial state of the sensor is HOLSTERED, do nothing.
                        if ((itSensors->lastState != UNKNOWN_STATE) || ((pSensorEvent->sensor_event != WEAPON_HOLSTERED) && (pSensorEvent->sensor_event != STUN_GUN_HOLSTERED)))
                        {
                            if ((dsi_read_last_ota_sensor_event_state() == OTA_SENSOR_EVENT_ENABLE) && (pSensorEvent->sensor_sequence_id != 0))
                            {
                                // Only queue new holster events when sensor OTA enabled
                                m_sensorData.push_back(SensorData(pSensorEvent->sensor_event, pSensorEvent->sensor_sequence_id, (UINT16)(pSensorEvent->requester_id)));

                                // Only update sequence ID when sensor OTA enabled
                                itSensors->sequenceId = pSensorEvent->sensor_sequence_id;
                            }

                            // Set oldest holster data in queue
                            setOldestSensorData();

                            m_sensorAlertRetries = MAX_SENSOR_ALERT_RETRIES;

                            // Update latest last state
                            itSensors->lastState = pSensorEvent->sensor_event;

                            // Initiate first element in queue
                            handleConflictingApps();
                        }
                    }
                }
                break;
            }
            // Handle sensor event Emergency action
            case AppBgsrvSensorEvent::SENSOR_EMERGENCY_ACTION:
            {
                sensor_emergency_app2app_msg_t* pSensorEmergency = sensorInput->getSensorEmergency();

                if ((pSensorEmergency != NULL) && (pSensorEmergency->emergency_type_id != UNKNOWN_TYPE_ID))
                {
                    if (pSensorEmergency->emergency_type_id == VEST_PIERCED)
                    {
                        switch (pSensorEmergency->emergency_type_event)
                        {
                            case VEST_PIERCED_UPPER:
                            case VEST_PIERCED_LOWER:
                            case VEST_PIERCED_UPPER_AND_LOWER:
                                launchVestPiercedEmergency();
                                break;

                            default:
                                break;
                        }
                    }
                }

                break;
            }
            case AppBgsrvSensorEvent::SENSOR_TSBK_RETRY_TIMER_EXPIRED:
            {
                // Reset and free timer
                manageSensorTimer(FREE_TIMER);

                handleConflictingApps();

                break;
            }
            case AppBgsrvSensorEvent::SENSOR_VC_STAT_UPDATE:
            {
                m_vcStat = sensorInput->getVcStat();

                if (m_vcStat != VOICE_CHANNEL_OCCUPIED)
                {
                    handleConflictingApps();
                }
                break;
            }
            // Receiving CNR_SNSR_ALERT_ON_CC bit from CL
            case AppBgsrvSensorEvent::SENSOR_ALERT_UPDATE:
            {
                dsi_reb_put_sensor_alert_supported(sensorInput->getSensorAlertSupported());

                // Set oldest holster data in queue
                setOldestSensorData();

                // Fall through
            }
            case AppBgsrvSensorEvent::SENSOR_HANDLE_CONFLICTING_APPS:
            {
                handleConflictingApps();

                if (m_relaunchVestPierced == true)
                {
                    // Relaunch vest pierced emergency once keyloading is detached and shutdown
                    launchVestPiercedEmergency();
                    m_relaunchVestPierced = false;
                }
                break;
            }
            // Receiving ACK status from CL
            case AppBgsrvSensorEvent::SENSOR_SIG_REQ_ACK_STATUS:
            {
                switch (sensorInput->getSigReqAckStatus())
                {
                    case GENERIC_SIG_REQ_SRV_CONFLICTED:
                    {
                        m_sigReqSrvConflicted = true;

                        break;
                    }
                    case GENERIC_SIG_REQ_SRV_CONFLICT_ENDED:
                    {
                        m_sigReqSrvConflicted = false;

                        // fall through to do retry
                    }
                    case GENERIC_SIG_REQ_DENY_BY_SCAN_ON:
                    case GENERIC_SIG_REQ_SRV_UNAVAILABLE:
                    case GENERIC_SIG_REQ_RETRY_EXHAUSTED:
                    case GENERIC_SIG_REQ_ACK_FAILED:
                    {
                        m_sensorEventMsgWaitForAck = false;

                        if ((m_sensorAlertRetries > 0) && (!m_sigReqSrvConflicted))
                        {
                            // Retry initialization of first element in queue
                            m_sensorAlertRetries--;

                            manageSensorTimer(START_TIMER);

                            break;
                        }
                        // fall through if finish retrying.
                    }
                    case GENERIC_SIG_REQ_ACK_RECEIVED:
                    {
                        m_sensorEventMsgWaitForAck = false;

                        if ((!m_sensorData.empty()) && (!m_sigReqSrvConflicted))
                        {
                            // Remove first element in queue
                            m_sensorData.pop_front();

                            // Reset oldest holster data in queue
                            setOldestSensorData();

                            // Reset retry number
                            m_sensorAlertRetries = MAX_SENSOR_ALERT_RETRIES;
                        }
                        // Initiate the next first element in queue
                        handleConflictingApps();

                        break;
                    }
                    case GENERIC_SIG_REQ_PROCEEDING:
                    case GENERIC_SIG_REQ_ECHO_RECEIVED:
                    case GENERIC_SIG_REQ_ECHO_FAILED:
                    case GENERIC_SIG_REQ_UNKNOWN_STATUS:
                    default:
                    {
                        // do nothing
                        break;
                    }
                }
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_INIT_DRAWN_STATE_SIGNALLING_SEQUENCE:
            {
                if (sensorInput->getSystemChanged() == TRUE)
                {
                    m_sensorData.clear();
                }

                if (dsi_read_last_ota_sensor_event_state() == OTA_SENSOR_EVENT_ENABLE)
                {
                    for (int i = 0; i < m_sensors.size(); i++)
                    {
                        // enqueue all drawn state sensor
                        if ((m_sensors[i].lastState == WEAPON_DRAWN) || (m_sensors[i].lastState == STUN_GUN_DRAWN))
                        {
                            std::list<SensorData>::iterator itSensorData = std::find_if(m_sensorData.begin(), m_sensorData.end(),
                            IsSameSensorEventAndTimId<SensorData>(m_sensors[i].lastState, m_sensors[i].timId));

                            if (itSensorData == m_sensorData.end())
                            {
                                m_sensorData.push_back(SensorData(m_sensors[i].lastState, m_sensors[i].sequenceId, m_sensors[i].timId));
                            }
                        }
                    }
                }

                m_sensorEventMsgWaitForAck = false;

                setOldestSensorData();

                m_sensorAlertRetries = MAX_SENSOR_ALERT_RETRIES;

                handleConflictingApps();

                if ((dsi_reb_get_app_active_status(APP_ACTIVE_EMERGENCY) == TRUE) && (m_vestPiercedEmergencyActive == true))
                {
                    // Set vest pierced emergency enabled if vest pierced emergency was previously active
                    dsi_reb_put_emergency_vest_pierced_enabled(TRUE);
                }
                else if (m_relaunchVestPierced == true)
                {
                    // Relaunch vest pierced emergency once mode change to a a valid channel mode
                    launchVestPiercedEmergency();
                }
                // Reset relaunch vest pierced flag
                m_relaunchVestPierced = false;
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_VEST_PIERCED_EMERGENCY_INACTIVE:
            {
                // Reset when app emergency is shut down or when other emergency source is triggered
                m_vestPiercedEmergencyActive = false;
                dsi_reb_put_emergency_vest_pierced_enabled(FALSE);
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_WAIT_FOR_SCAN_OFF:
            {
                // wait here for APP_SCAN_ACTIVE to be shutdown completely and inform us from APP_DISPATCH_SCAN.
                m_needToResumeScan = true;
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_INIT_SIGNALLING_SEQUENCE:
            {
                initSigSequence();
                break;
            }
            case AppBgsrvSensorEvent::SENSOR_UNKNOWN_INPUT:
            default:
            {
                // do nothing.
                break;
            }
        }
    }
    return controllerDone;
}

//--------------------------------------------------------------------------------
//  Function:     initSigSequence
//
//  Abstract
//      This function is to initiate the signalling sequence.
//--------------------------------------------------------------------------------
void SensorController::initSigSequence()
{
    if (isInitSigSequenceAllowed())
    {
        sl_crt_snsr_alert_usr_data_t snr_alrt_info;
        sl_generic_sig_req_initiate_signalling_t * msg_ptr = (sl_generic_sig_req_initiate_signalling_t*)Get_Buf(GET_POOL_ID(BUFSIZE(sl_generic_sig_req_initiate_signalling_t)));

        if (msg_ptr != NULL)
        {
            snr_alrt_info.sequence_id = m_sensorData.front().sensorSequenceId;
            snr_alrt_info.event = m_sensorData.front().sensorEvent;
            ((sl_generic_sig_req_initiate_signalling_t*)msg_ptr )->generic_sig_req_intf = create_SensorAlertSigReq(snr_alrt_info);

            // vris_msg_t header will be filled in sl_crt_snsr_alert_sig_req() function
            svcClimSendDataToCl(0, (BUF_PTR)msg_ptr, (climDataReqFuncPtr_t)sl_crt_snsr_alert_sig_req, getAppId(), OP_SLIO_SUBMODE_CTRL);
            m_sensorEventMsgWaitForAck = true;
        }
    }
    else if ((m_needToResumeScan == true) && (m_sensorEventMsgWaitForAck == false))
    {
        // Resume scan if required when not initialising signalling's sequence
        app_scan_active_launch(getAppId(), FALSE);
        m_needToResumeScan = false;
    }
}

//--------------------------------------------------------------------------------
//  Function:     isInitSigSequenceAllowed
//
//  Abstract
//      This is a function that checks if initiation of signalling sequence is allowed.
//--------------------------------------------------------------------------------
bool SensorController::isInitSigSequenceAllowed()
{
    return ((m_sigReqSrvConflicted == false) &&                                                                  // check if signalling service conflicted
            (m_sensorEventMsgWaitForAck == false) &&                                                             // check if waiting for ACK
            (dsi_util_get_current_sig_type() == SIG_APCO_TRUNKING) &&                                            // check if current signalling type apco trunk
            (dsi_read_tx_inhibit_enabled() == FALSE) &&                                                          // check if tx inhibit is enabled
            ((svcHEBGetCallState() == TRUE) ||                                                                   // check if Remote Monitor is active OR
             ((svcHEBGetRadioTxing() == FALSE) &&                                                                // check if radio is txing
              ((m_vcStat != VOICE_CHANNEL_OCCUPIED) ||                                                           // check if voice channel is occupied OR
               (dsi_reb_get_call_state(XCMP_CALL_CTRL_FEATURE_PHONE) == XCMP_CALL_CTRL_STATE_IN_PROGRESS)))) &&  // check if the phone call is in progress
            (dsi_reb_get_sensor_alert_supported() == TRUE) &&                                                    // check if radio support TSBK
            (dsi_reb_get_is_ext_kyld_connected() == FALSE) &&                                                    // check if external keyload is attached
            (dsi_reb_is_emergency_busy() == FALSE) &&                                                            // check if emergency is busy
            (dsi_read_last_ota_sensor_event_state() == OTA_SENSOR_EVENT_ENABLE) &&                               // check if sensor disarm is enabled
            (m_tsbkRetryTimerHandler == 0) &&                                                                    // check if TSBK retry timer is freed
            (dsi_reb_get_oor() == FALSE) &&                                                                      // check if radio is out of range
            (dsi_reb_get_failsoft() == FALSE) &&                                                                 // check if radio is in failsoft state
            (!m_sensorData.empty()));                                                                            // check if holster data in queue is empty
}

//--------------------------------------------------------------------------------
//  Function:     handleConflictingApps
//
//  Abstract
//      This is a function that handles sensor controller feature interaction if
//      initiation of signalling sequence is allowed.
//--------------------------------------------------------------------------------
void SensorController::handleConflictingApps()
{
    if (isInitSigSequenceAllowed())
    {
        // Sensor critical alert broadcast app2app interaction shutdown
        svcLogEntryInAppMsgQueue(NULL_APPID,                                    // target app id
                                 getAppId(),                                    // sender app id
                                 FALSE,                                         // notify
                                 FALSE,                                         // activate target
                                 SENSOR_EVENT_SHUTDOWN_CONFLICTING_APPS,        // msg opcode
                                 NULL);                                         // data ptr

        // if APP_SCAN_ACTIVE is on shut it down before initialising signalling sequence
        app_scan_active_shutdown(getAppId());
    }
    else if ((m_needToResumeScan == true) && (m_sensorEventMsgWaitForAck == false))
    {
        // Resume scan if required when not initialising signalling's sequence
        app_scan_active_launch(getAppId(), FALSE);
        m_needToResumeScan = false;
    }
}

//--------------------------------------------------------------------------------
//  Function:     setOldestSensorData
//
//  Abstract
//      This is a function that uses vr_env_put to set the oldest sensor event and
//      its sequence id in the queue.
//--------------------------------------------------------------------------------
void SensorController::setOldestSensorData()
{
    if ((dsi_reb_get_sensor_alert_supported() == TRUE) && (!m_sensorData.empty()))
    {
        vr_env_put_sensor_oldest_event(m_sensorData.front().sensorEvent);
        vr_env_put_sensor_oldest_sequence_id(m_sensorData.front().sensorSequenceId);
    }
    else
    {
        vr_env_put_sensor_oldest_event(UNKNOWN_STATE);
        vr_env_put_sensor_oldest_sequence_id(0);
    }
}

//--------------------------------------------------------------------------------
//  Function:     launchVestPiercedEmergency
//
//  Abstract
//      This is a function that uses to launch emergency from background service app.
//--------------------------------------------------------------------------------
void SensorController::launchVestPiercedEmergency()
{
    boolean silent_emergency_enabled = dsi_read_is_silent_emer_enabled();
    bool launch_success = false;

    if (dsi_read_is_emergency_enabled() == TRUE)
    {
        if ((svcHEBGetModeInvalid() == TRUE) || (dsi_reb_get_is_ext_kyld_connected() == TRUE))
        {
            // Radio is currently in an invalid mode: Unprogrammed Channel, Blank Channel, Wrong Rx/Tx Freq OR keyloading attached
            // Do not launch Vest Pierced emergency until mode change to a valid mode OR keyloading detached
            m_relaunchVestPierced = true;
        }
        else
        {
            // Set emergency vest pierced enabled
            dsi_reb_put_emergency_vest_pierced_enabled(TRUE);

            // Save the trigger that launch the emergency application
            svcHEBPutEmerTriggerSource(TRIGGER_SOURCE_VEST_PIERCED);

            if (dsi_reb_get_app_active_status(APP_ACTIVE_EMERGENCY) == FALSE)
            {
                svcLogEntryInAppMsgQueue(APP_EMERGENCY,            // target app id
                                         APP_BACKGROUND_SERVICE,   // sender's app id
                                         FALSE,                    // notify
                                         TRUE,                     // activate target
                                         EMERGENCY_LAUNCH,         // msg opcode
                                         NULL);                    // data ptr
            }
            else // Emergency app already active, vest pierced emergency needs to replace other emergency trigger source
            {
                svcLogEntryInAppMsgQueue(APP_EMERGENCY,            // target app id
                                         APP_BACKGROUND_SERVICE,   // sender's app id
                                         FALSE,                    // notify
                                         FALSE,                    // activate target
                                         EMERGENCY_VEST_PIERCED,   // msg opcode
                                         NULL);                    // data ptr
            }
            launch_success = true;
            m_vestPiercedEmergencyActive = true;
        }
    }

    if (silent_emergency_enabled == FALSE)
    {
        if ((launch_success == true) && (dsi_reb_get_app_active_status(APP_ACTIVE_EMERGENCY) == FALSE))
        {
            svcRequestCommonTone(getAppId(), VALID_INPUT_TONE, TONE_ON);
        }
        else if (launch_success == false)
        {
            svcRequestCommonTone(getAppId(), INVALID_INPUT_TONE, TONE_ON);
        }
    }
}

//--------------------------------------------------------------------------------
//  Function:     getRearmTriggerRate
//
//  Abstract
//      This is a function that returns the rearmTriggerRate to set the no report timer
//      in seconds for Weapon Fired and Vest Pierced sensors.
//--------------------------------------------------------------------------------
uint16_t SensorController::getRearmTriggerRate(uint16_t tim_id)
{
    if (((tim_id & 0xFFF0) == TEDS_ID_YARDARM) ||     // Yardarm weapon fired sensor
        ((tim_id & 0xFFF0) == TEDS_ID_VEST_PIERCED))  // Vest pierced sensor
    {
        return 15; // rearm every 15 seconds
    }
    else // Gun and Taser Holster sensor
    {
        return 0; // permanent rearm
    }
}
