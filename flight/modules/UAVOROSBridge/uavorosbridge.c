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
/*
#include "receiverstatus.h"
#include "hwsettings.h"
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
#include "taskinfo.h"
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









typedef enum {
    ROS_IDLE,
} ros_state;



struct ros_bridge {
    uintptr_t    com;

    ros_state    state;
    size_t 
    uint8_t ros_tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
    uint8_t ros_rx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
};

#if defined(PIOS_ROS_STACK_SIZE)
#define STACK_SIZE_BYTES     PIOS_ROS_STACK_SIZE
#else
#define STACK_SIZE_BYTES     768
#endif
#define TASK_PRIORITY        (tskIDLE_PRIORITY)

#define MAX_ALARM_LEN        30

#define BOOT_DISPLAY_TIME_MS (10 * 1000)

static bool module_enabled = false;
static struct ros_bridge *ros;
static int32_t uavoROSBridgeInitialize(void);
static void uavoROSBridgeTask(void *parameters);

static void ros_send(struct ros_bridge *m, uint8_t cmd, const uint8_t *data, size_t len)
{
    uint8_t buf[5];
    uint8_t cs = (uint8_t)(len) ^ cmd;

    buf[0] = '$';
    buf[1] = 'M';
    buf[2] = '>';
    buf[3] = (uint8_t)(len);
    buf[4] = cmd;

    PIOS_COM_SendBuffer(m->com, buf, sizeof(buf));

    if (len > 0) {
        PIOS_COM_SendBuffer(m->com, data, len);

        for (unsigned i = 0; i < len; i++) {
            cs ^= data[i];
        }
    }

    cs    ^= 0;

    buf[0] = cs;
    PIOS_COM_SendBuffer(m->com, buf, 1);
}

/**
 * Process incoming bytes from an ROS query thing.
 * @param[in] b received byte
 * @return true if we should continue processing bytes
 */
static bool ros_receive_byte(struct ros_bridge *m, uint8_t b)
{
    switch (m->state) {
    case ROS_IDLE:
        switch (b) {
        case 0xe0: // uavtalk matching first part of 0x3c @ 57600 baud
            m->state = ROS_MAYBE_UAVTALK_SLOW2;
            break;
        case '<': // uavtalk matching with 0x3c 0x2x 0xxx 0x0x
            m->state = ROS_MAYBE_UAVTALK2;
            break;
        case '$':
            m->state = ROS_HEADER_START;
            break;
        default:
            m->state = ROS_IDLE;
        }
        break;
    case ROS_HEADER_START:
        m->state = b == 'M' ? ROS_HEADER_M : ROS_IDLE;
        break;
    case ROS_HEADER_M:
        m->state = b == '<' ? ROS_HEADER_SIZE : ROS_IDLE;
        break;
    case ROS_HEADER_SIZE:
        m->state = ros_state_size(m, b);
        break;
    case ROS_HEADER_CMD:
        m->state = ros_state_cmd(m, b);
        break;
    case ROS_FILLBUF:
        m->state = ros_state_fill_buf(m, b);
        break;
    case ROS_CHECKSUM:
        m->state = ros_state_checksum(m, b);
        break;
    case ROS_DISCARD:
        m->state = ros_state_discard(m, b);
        break;
    case ROS_MAYBE_UAVTALK2:
        // e.g. 3c 20 1d 00
        // second possible uavtalk byte
        m->state = (b & 0xf0) == 0x20 ? ROS_MAYBE_UAVTALK3 : ROS_IDLE;
        break;
    case ROS_MAYBE_UAVTALK3:
        // third possible uavtalk byte can be anything
        m->state = ROS_MAYBE_UAVTALK4;
        break;
    case ROS_MAYBE_UAVTALK4:
        m->state = ROS_IDLE;
        // If this looks like the fourth possible uavtalk byte, we're done
        if ((b & 0xf0) == 0) {
            PIOS_COM_TELEM_RF = m->com;
            return false;
        }
        break;
    case ROS_MAYBE_UAVTALK_SLOW2:
        m->state = b == 0x18 ? ROS_MAYBE_UAVTALK_SLOW3 : ROS_IDLE;
        break;
    case ROS_MAYBE_UAVTALK_SLOW3:
        m->state = b == 0x98 ? ROS_MAYBE_UAVTALK_SLOW4 : ROS_IDLE;
        break;
    case ROS_MAYBE_UAVTALK_SLOW4:
        m->state = b == 0x7e ? ROS_MAYBE_UAVTALK_SLOW5 : ROS_IDLE;
        break;
    case ROS_MAYBE_UAVTALK_SLOW5:
        m->state = b == 0x00 ? ROS_MAYBE_UAVTALK_SLOW6 : ROS_IDLE;
        break;
    case ROS_MAYBE_UAVTALK_SLOW6:
        m->state = ROS_IDLE;
        // If this looks like the sixth possible 57600 baud uavtalk byte, we're done
        if (b == 0x60) {
            PIOS_COM_ChangeBaud(m->com, 57600);
            PIOS_COM_TELEM_RF = m->com;
            return false;
        }
        break;
    }

    return true;
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
            PIOS_COM_TELEM_RF = pios_com_ros_id;
        }

        return -1;
    }

    xTaskHandle taskHandle;

    xTaskCreate(uavoROSBridgeTask, "uavoROSBridge", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_UAVOROSBRIDGE, taskHandle);

    return 0;
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
 * Module initialization routine
 * @return 0 when initialization was successful
 */
static int32_t uavoROSBridgeInitialize(void)
{
    if (pios_com_ros_id) {
        ros = pios_malloc(sizeof(*ros));
        if (ros != NULL) {
            memset(ros, 0x00, sizeof(*ros));

            ros->com = pios_com_ros_id;

            // now figure out enabled features: registered sensors, ADC routing, GPS

            HwSettingsROSSpeedOptions rosSpeed;
            HwSettingsROSSpeedGet(&rosSpeed);

            PIOS_COM_ChangeBaud(pios_com_ros_id, hwsettings_rosspeed_enum_to_baud(rosSpeed));

            module_enabled = true;
            return 0;
        }
    }

    return -1;
}
MODULE_INITCALL(uavoROSBridgeInitialize, uavoROSBridgeStart);

/**
 * Main task routine
 * @param[in] parameters parameter given by PIOS_Thread_Create()
 */
static void uavoROSBridgeTask(__attribute__((unused)) void *parameters)
{
    while (1) {
        uint8_t b = 0;
        uint16_t count = PIOS_COM_ReceiveBuffer(ros->com, &b, 1, ~0);
        if (count) {
            if (!ros_receive_byte(ros, b)) {
                // Returning is considered risky here as
                // that's unusual and this is an edge case.
                while (1) {
                    PIOS_DELAY_WaitmS(60 * 1000);
                }
            }
        }
    }
}

#endif // PIOS_INCLUDE_ROS_BRIDGE
/**
 * @}
 * @}
 */
