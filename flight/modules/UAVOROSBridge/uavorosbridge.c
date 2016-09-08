/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       uavorosridge.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             Max Planck Institute for intelligent systems, http://www.is.mpg.de Copyright (C) 2016.
 * @brief      Bridges certain UAVObjects to ROS on USB VCP
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "openpilot.h"
#include "hwsettings.h"
#include "taskinfo.h"
#include "callbackinfo.h"
/*
   #include "receiverstatus.h"
   #include "flightmodesettings.h"
   #include "flightbatterysettings.h"
   #include "flightbatterystate.h"
   #include "gpspositionsensor.h"
   #include "manualcontrolcommand.h"
   #include "manualcontrolsettings.h"
   #include "oplinkstatus.h"
   #include "accessorydesired.h"
   #include "attitudestate.h"
   #include "airspeedstate.h"
   #include "actuatorsettings.h"
   #include "actuatordesired.h"
   #include "flightstatus.h"
   #include "systemstats.h"
   #include "systemalarms.h"
   #include "takeofflocation.h"
   #include "homelocation.h"
   #include "positionstate.h"
   #include "velocitystate.h"
   #include "stabilizationdesired.h"
   #include "stabilizationsettings.h"
   #include "stabilizationbank.h"
   #include "stabilizationsettingsbank1.h"
   #include "stabilizationsettingsbank2.h"
   #include "stabilizationsettingsbank3.h"
   #include "magstate.h"
 */
#include "objectpersistence.h"

#include "pios_sensors.h"

#if defined(PIOS_INCLUDE_ROS_BRIDGE)

#include <uavorosbridgemessage_priv.h>

struct ros_bridge {
    uintptr_t     com;

    uint32_t      lastPingTimestamp;
    uint8_t       pingSequence;
    PiOSDeltatimeConfig roundtrip;
    double        roundTripTime;
    uint8_t       rx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
    size_t        rx_length;
    volatile bool scheduled[ROSBRIDGEMESSAGE_END_ARRAY_SIZE];
};

#if defined(PIOS_ROS_STACK_SIZE)
#define STACK_SIZE_BYTES  PIOS_ROS_STACK_SIZE
#else
#define STACK_SIZE_BYTES  768
#endif
#define TASK_PRIORITY     CALLBACK_TASK_AUXILIARY
#define CALLBACK_PRIORITY CALLBACK_PRIORITY_REGULAR
#define CBTASK_PRIORITY   CALLBACK_TASK_AUXILIARY

static bool module_enabled = false;
static struct ros_bridge *ros;
static int32_t uavoROSBridgeInitialize(void);
static void uavoROSBridgeRxTask(void *parameters);
static void uavoROSBridgeTxTask(void);
static DelayedCallbackInfo *callbackHandle;
static rosbridgemessage_handler ping_handler, pong_handler, fullstate_estimate_handler, imu_average_handler, gimbal_estimate_handler;

static rosbridgemessage_handler *const rosbridgemessagehandlers[ROSBRIDGEMESSAGE_END_ARRAY_SIZE] = {
    ping_handler,
    NULL,
    NULL,
    NULL,
    pong_handler,
    fullstate_estimate_handler,
    imu_average_handler,
    gimbal_estimate_handler
};


/**
 * Process incoming bytes from an ROS query thing.
 * @param[in] b received byte
 * @return true if we should continue processing bytes
 */
static void ros_receive_byte(struct ros_bridge *m, uint8_t b)
{
    m->rx_buffer[m->rx_length] = b;
    m->rx_length++;
    rosbridgemessage_t *message = (rosbridgemessage_t *)m->rx_buffer;

    // very simple parser - but not a state machine, just a few checks
    if (m->rx_length <= offsetof(rosbridgemessage_t, length)) {
        // check (partial) magic number - partial is important since we need to restart at any time if garbage is received
        uint32_t canary = 0xff;
        for (uint32_t t = 1; t < m->rx_length; t++) {
            canary = (canary << 8) || 0xff;
        }
        if ((message->magic & canary) != (ROSBRIDGEMAGIC & canary)) {
            // parse error, not beginning of message
            m->rx_length = 0;
        }
    }
    if (m->rx_length == offsetof(rosbridgemessage_t, timestamp)) {
        if (message->length < offsetof(rosbridgemessage_t, data) || message->length > ROSBRIDGEMESSAGE_BUFFERSIZE - offsetof(rosbridgemessage_t, data)) {
            // parse error, no messages are that long
            m->rx_length = 0;
        }
    }
    if (m->rx_length == offsetof(rosbridgemessage_t, crc32)) {
        if (message->type >= ROSBRIDGEMESSAGE_END_ARRAY_SIZE) {
            // parse error
            m->rx_length = 0;
        }
        if (message->length != ROSBRIDGEMESSAGE_SIZES[message->type]) {
            // parse error
            m->rx_length = 0;
        }
    }
    if (m->rx_length < offsetof(rosbridgemessage_t, data)) {
        // not a parse failure, just not there yet
        return;
    }
    if (m->rx_length == offsetof(rosbridgemessage_t, data) + ROSBRIDGEMESSAGE_SIZES[message->type]) {
        // complete message received and stored in pointer "message"
        switch (message->type) {
        case ROSBRIDGEMESSAGE_PING:
            m->scheduled[ROSBRIDGEMESSAGE_PONG];
            PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
            break;
        case ROSBRIDGEMESSAGE_POSVEL_ESTIMATE:
            // TODO tell SateEstimation a position and velocity including variance
            break;
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL:
            // TODO set apropriate UAVObjects for fliht control overrides
            break;
        case ROSBRIDGEMESSAGE_GIMBALCONTROL:
            // TODO implement gimbal control somehow
            break;
        case ROSBRIDGEMESSAGE_PONG:
            pong_handler(m, message);
            break;
        default:
            // do nothing at all and discard the message
            break;
        }
    }
}

static uint32_t hwsettings_rosspeed_enum_to_baud(uint8_t baud)
{
    switch (baud) {
    case HWSETTINGS_ROSSPEED_2400:
        return 2400;

    case HWSETTINGS_ROSSPEED_4800:
        return 4800;

    case HWSETTINGS_ROSSPEED_9600:
        return 9600;

    case HWSETTINGS_ROSSPEED_19200:
        return 19200;

    case HWSETTINGS_ROSSPEED_38400:
        return 38400;

    case HWSETTINGS_ROSSPEED_57600:
        return 57600;

    default:
    case HWSETTINGS_ROSSPEED_115200:
        return 115200;
    }
}


/**
 * Module start routine automatically called after initialization routine
 * @return 0 when was successful
 */
static int32_t uavoROSBridgeStart(void)
{
    if (!module_enabled) {
        // give port to telemetry if it doesn't have one
        // stops board getting stuck in condition where it can't be connected to gcs
        if (!PIOS_COM_TELEM_RF) {
            PIOS_COM_TELEM_RF = PIOS_COM_ROS;
        }

        return -1;
    }

    PIOS_DELTATIME_Init(&ros->roundtrip, 1e-3f, 1e-6f, 10.0f, 1e-1f);

    xTaskHandle taskHandle;

    xTaskCreate(uavoROSBridgeRxTask, "uavoROSBridge", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_UAVOROSBRIDGE, taskHandle);
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);

    return 0;
}

/**
 * Module initialization routine
 * @return 0 when initialization was successful
 */
static int32_t uavoROSBridgeInitialize(void)
{
    if (PIOS_COM_ROS) {
        ros = pios_malloc(sizeof(*ros));
        if (ros != NULL) {
            memset(ros, 0x00, sizeof(*ros));

            ros->com = PIOS_COM_ROS;

            HwSettingsInitialize();
            HwSettingsROSSpeedOptions rosSpeed;
            HwSettingsROSSpeedGet(&rosSpeed);

            PIOS_COM_ChangeBaud(PIOS_COM_ROS, hwsettings_rosspeed_enum_to_baud(rosSpeed));
            callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&uavoROSBridgeTxTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_UAVOROSBRIDGE, STACK_SIZE_BYTES);

            module_enabled = true;
            return 0;
        }
    }

    return -1;
}
MODULE_INITCALL(uavoROSBridgeInitialize, uavoROSBridgeStart);

/** various handlers **/
static void ping_handler(struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pingpong_t *data = (rosbridgemessage_pingpong_t *)&(m->data);

    data->sequence_number = rb->pingSequence++;
    rb->roundtrip.last    = PIOS_DELAY_GetRaw();
}

static void pong_handler(struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pingpong_t *data = (rosbridgemessage_pingpong_t *)&(m->data);

    if (data->sequence_number != rb->pingSequence) {
        return;
    }
    PIOS_DELTATIME_GetAverageSeconds(&(rb->roundtrip));
}
static void fullstate_estimate_handler(__attribute__((unused)) struct ros_bridge *rb, __attribute__((unused)) rosbridgemessage_t *m)
{
    // TODO
}
static void imu_average_handler(__attribute__((unused)) struct ros_bridge *rb, __attribute__((unused)) rosbridgemessage_t *m)
{
    // TODO
}
static void gimbal_estimate_handler(__attribute__((unused)) struct ros_bridge *rb, __attribute__((unused)) rosbridgemessage_t *m)
{
    // TODO
}

/**
 * Main task routine
 * @param[in] parameters parameter given by PIOS_Callback_Create()
 */
static void uavoROSBridgeTxTask(void)
{
    uint8_t buffer[ROSBRIDGEMESSAGE_BUFFERSIZE]; // buffer on the stack? could also be in static RAM but not reuseale by other callbacks then
    rosbridgemessage_t *message = (rosbridgemessage_t *)buffer;

    for (rosbridgemessagetype_t type = ROSBRIDGEMESSAGE_PING; type < ROSBRIDGEMESSAGE_END_ARRAY_SIZE; type++) {
        if (ros->scheduled[type] && rosbridgemessagehandlers[type] != NULL) {
            message->magic     = ROSBRIDGEMAGIC;
            message->length    = ROSBRIDGEMESSAGE_SIZES[type];
            message->type      = type;
            message->timestamp = PIOS_DELAY_GetuS();
            (*rosbridgemessagehandlers[type])(ros, message);
            message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
            int32_t ret = PIOS_COM_SendBufferNonBlocking(ros->com, buffer, offsetof(rosbridgemessage_t, data) + message->length);
            if (ret >= 0) {
                ros->scheduled[type] = false;
            }
            PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
            return;
        }
    }
    // nothing scheduled, do a ping in 10 secods time

ros->scheduled[ROSBRIDGEMESSAGE_PING] = true;
    PIOS_CALLBACKSCHEDULER_Schedule(callbackHandle,10000,CALLBACK_UPDATEMODE_SOONER);
}


/**
 * Main task routine
 * @param[in] parameters parameter given by PIOS_Thread_Create()
 */
static void uavoROSBridgeRxTask(__attribute__((unused)) void *parameters)
{
    while (1) {
        uint8_t b = 0;
        uint16_t count = PIOS_COM_ReceiveBuffer(ros->com, &b, 1, ~0);
        if (count) {
            ros_receive_byte(ros, b);
        }
    }
}

#endif // PIOS_INCLUDE_ROS_BRIDGE
/**
 * @}
 * @}
 */
