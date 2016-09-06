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




// message format

struct ros_bridge_message {
    



}

typedef 













typedef enum {
    ROS_IDLE,
    ROS_HEADER_START,
    ROS_HEADER_M,
    ROS_HEADER_SIZE,
    ROS_HEADER_CMD,
    ROS_FILLBUF,
    ROS_CHECKSUM,
    ROS_DISCARD,
    ROS_MAYBE_UAVTALK2,
    ROS_MAYBE_UAVTALK3,
    ROS_MAYBE_UAVTALK4,
    ROS_MAYBE_UAVTALK_SLOW2,
    ROS_MAYBE_UAVTALK_SLOW3,
    ROS_MAYBE_UAVTALK_SLOW4,
    ROS_MAYBE_UAVTALK_SLOW5,
    ROS_MAYBE_UAVTALK_SLOW6
} ros_state;



struct ros_bridge {
    uintptr_t    com;

    uint8_t      sensors;
    uint8_t      analog;

    UAVObjHandle current_pid_bank;

    ros_state    state;
    uint8_t      cmd_size;
    uint8_t      cmd_id;
    uint8_t      cmd_i;
    uint8_t      checksum;
    union {
        uint8_t   data[0];
        // Specific packed data structures go here.
        ros_pid_t piditems[PID_ITEM_COUNT];
    } cmd_data;
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

static ros_state ros_state_size(struct ros_bridge *m, uint8_t b)
{
    m->cmd_size = b;
    m->checksum = b;
    return ROS_HEADER_CMD;
}

static ros_state ros_state_cmd(struct ros_bridge *m, uint8_t b)
{
    m->cmd_i     = 0;
    m->cmd_id    = b;
    m->checksum ^= m->cmd_id;

    if (m->cmd_size > sizeof(m->cmd_data)) {
        // Too large a body.  Let's ignore it.
        return ROS_DISCARD;
    }

    return m->cmd_size == 0 ? ROS_CHECKSUM : ROS_FILLBUF;
}

static ros_state ros_state_fill_buf(struct ros_bridge *m, uint8_t b)
{
    m->cmd_data.data[m->cmd_i++] = b;
    m->checksum ^= b;
    return m->cmd_i == m->cmd_size ? ROS_CHECKSUM : ROS_FILLBUF;
}

static void ros_send_attitude(struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            int16_t x;
            int16_t y;
            int16_t h;
        } att;
    } data;
    AttitudeStateData attState;

    AttitudeStateGet(&attState);

    // Roll and Pitch are in 10ths of a degree.
    data.att.x = attState.Roll * 10;
    data.att.y = attState.Pitch * -10;
    // Yaw is just -180 -> 180
    data.att.h = attState.Yaw;

    ros_send(m, ROS_ATTITUDE, data.buf, sizeof(data));
}

#define IS_STAB_MODE(d, m) (((d).Roll == (m)) && ((d).Pitch == (m)))

static void ros_send_status(struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            uint16_t cycleTime;
            uint16_t i2cErrors;
            uint16_t sensors;
            uint32_t flags;
            uint8_t  setting;
        } __attribute__((packed)) status;
    } data;
    // TODO: https://github.com/TauLabs/TauLabs/blob/next/shared/uavobjectdefinition/actuatordesired.xml#L8
    data.status.cycleTime = 0;
    data.status.i2cErrors = 0;

    data.status.sensors   = m->sensors;

    if (GPSPositionSensorHandle() != NULL) {
        GPSPositionSensorStatusOptions gpsStatus;
        GPSPositionSensorStatusGet(&gpsStatus);

        if (gpsStatus != GPSPOSITIONSENSOR_STATUS_NOGPS) {
            data.status.sensors |= ROS_SENSOR_GPS;
        }
    }

    data.status.flags   = 0;
    data.status.setting = 0;

    FlightStatusData flightStatus;
    FlightStatusGet(&flightStatus);

    StabilizationDesiredStabilizationModeData stabModeData;
    StabilizationDesiredStabilizationModeGet(&stabModeData);

    if (flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED) {
        data.status.flags |= (1 << ROS_STATUS_ARMED);
    }

    // flightmode

    if (flightStatus.FlightMode == FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD) {
        data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_GPSHOLD);
    } else if (flightStatus.FlightMode == FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE) {
        data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_GPSHOME);
    } else if (flightStatus.FlightMode == FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER) {
        data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_GPSMISSION);
    } else if (flightStatus.FlightMode == FLIGHTSTATUS_FLIGHTMODE_LAND) {
        data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_GPSLAND);
    } else if (flightStatus.FlightMode == FLIGHTSTATUS_FLIGHTMODE_MANUAL) {
        data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_PASSTHRU);
    } else { //
             // if Roll(Rate) && Pitch(Rate) => Acro
             // if Roll(Acro+) && Pitch(Acro+) => Acro+
             // if Roll(Rattitude) && Pitch(Rattitude) => Horizon
             // else => Stabilized

        if (IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_RATE)
            || IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_RATETRAINER)) {
            // data.status.flags should not set any mode flags
        } else if (IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_ACRO)) { // this is Acro+ mode
            data.status.flags |= (1 << ROS_STATUS_ICON_ACROPLUS);
        } else if (IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_RATTITUDE)) {
            data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_HORIZON);
        } else { // looks like stabilized mode, whichever it is..
            data.status.flags |= (1 << ROS_STATUS_FLIGHTMODE_STABILIZED);
        }
    }

    // sensors in use?
    // flight mode HORIZON or STABILIZED will turn on Accelerometer icon.
    // Barometer?

    if ((stabModeData.Thrust == STABILIZATIONDESIRED_STABILIZATIONMODE_ALTITUDEHOLD)
        || (stabModeData.Thrust == STABILIZATIONDESIRED_STABILIZATIONMODE_ALTITUDEVARIO)) {
        data.status.flags |= (1 << ROS_STATUS_SENSOR_BARO);
    }

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    if (MagStateHandle() != NULL) {
        MagStateSourceOptions magSource;
        MagStateSourceGet(&magSource);

        if (magSource != MAGSTATE_SOURCE_INVALID) {
            data.status.flags |= (1 << ROS_STATUS_SENSOR_MAG);
        }
    }
#endif /* PIOS_EXCLUDE_ADVANCED_FEATURES */

    if (flightStatus.AlwaysStabilizeWhenArmed == FLIGHTSTATUS_ALWAYSSTABILIZEWHENARMED_TRUE) {
        data.status.flags |= (1 << ROS_STATUS_ICON_AIRMODE);
    }

    ros_send(m, ROS_STATUS, data.buf, sizeof(data));
}

static void ros_send_analog(struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            uint8_t  vbat;
            uint16_t powerMeterSum;
            uint16_t rssi;
            uint16_t current;
        } __attribute__((packed)) status;
    } data;

    memset(&data, 0, sizeof(data));

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    if (FlightBatteryStateHandle() != NULL) {
        FlightBatteryStateData batState;
        FlightBatteryStateGet(&batState);

        if (m->analog & ROS_ANALOG_VOLTAGE) {
            data.status.vbat = (uint8_t)lroundf(batState.Voltage * 10);
        }
        if (m->analog & ROS_ANALOG_CURRENT) {
            data.status.current = lroundf(batState.Current * 100);
            data.status.powerMeterSum = lroundf(batState.ConsumedEnergy);
        }
    }
#endif
    // OPLink supports RSSI reported in dBm, but updates different value in ReceiverStatus.Quality (link_quality - function of lost, repaired and good packet count).
    // We use same method as in Receiver module to decide if oplink is used for control:
    // Check for Throttle channel group, if this belongs to oplink, we will use oplink rssi instead of ReceiverStatus.Quality.

    ManualControlSettingsChannelGroupsData channelGroups;
    ManualControlSettingsChannelGroupsGet(&channelGroups);

#ifdef PIOS_INCLUDE_OPLINKRCVR
    if (channelGroups.Throttle == MANUALCONTROLSETTINGS_CHANNELGROUPS_OPLINK) {
        int8_t rssi;
        OPLinkStatusRSSIGet(&rssi);

        // ROS values have no units, and OSD rssi display requires calibration anyway, so we will translate OPLINK_LOW_RSSI to OPLINK_HIGH_RSSI -> 0-1023
        if (rssi < OPLINK_LOW_RSSI) {
            rssi = OPLINK_LOW_RSSI;
        } else if (rssi > OPLINK_HIGH_RSSI) {
            rssi = OPLINK_HIGH_RSSI;
        }

        data.status.rssi = ((rssi - OPLINK_LOW_RSSI) * 1023) / (OPLINK_HIGH_RSSI - OPLINK_LOW_RSSI);
    } else {
#endif /* PIOS_INCLUDE_OPLINKRCVR */
    uint8_t quality;
    ReceiverStatusQualityGet(&quality);

    // ROS RSSI's range is 0-1023
    data.status.rssi = (quality * 1023) / 100;
#ifdef PIOS_INCLUDE_OPLINKRCVR
}
#endif /* PIOS_INCLUDE_OPLINKRCVR */

    if (data.status.rssi > 1023) {
        data.status.rssi = 1023;
    }

    ros_send(m, ROS_ANALOG, data.buf, sizeof(data));
}

static void ros_send_raw_gps(struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            uint8_t  fix;                 // 0 or 1
            uint8_t  num_sat;
            int32_t  lat;                  // 1 / 10 000 000 deg
            int32_t  lon;                  // 1 / 10 000 000 deg
            uint16_t alt; // meter
            uint16_t speed; // cm/s
            int16_t  ground_course;        // degree * 10
        } __attribute__((packed)) raw_gps;
    } data;

    GPSPositionSensorData gps_data;

    if (GPSPositionSensorHandle() != NULL) {
        GPSPositionSensorGet(&gps_data);

        data.raw_gps.fix     = (gps_data.Status >= GPSPOSITIONSENSOR_STATUS_FIX2D ? 1 : 0);  // Data will display on OSD if 2D fix or better
        data.raw_gps.num_sat = gps_data.Satellites;
        data.raw_gps.lat     = gps_data.Latitude;
        data.raw_gps.lon     = gps_data.Longitude;
        data.raw_gps.alt     = (uint16_t)gps_data.Altitude;
        data.raw_gps.speed   = (uint16_t)gps_data.Groundspeed;
        data.raw_gps.ground_course = (int16_t)(gps_data.Heading * 10.0f);

        ros_send(m, ROS_RAW_GPS, data.buf, sizeof(data));
    }
}

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES

static inline float Sq(float x)
{
    return x * x;
}

static void ros_send_comp_gps(struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            uint16_t distance_to_home; // meter
            int16_t  direction_to_home;    // degree [-180:180]
            uint8_t  home_position_valid;  // 0 = Invalid, Dronin ROS specific
        } __attribute__((packed)) comp_gps;
    } data;

    data.comp_gps.distance_to_home    = 0;
    data.comp_gps.direction_to_home   = 0;
    data.comp_gps.home_position_valid = 0; // Home distance and direction will not display on OSD

    if (PositionStateHandle() && TakeOffLocationHandle()) {
        TakeOffLocationData takeOffLocation;
        TakeOffLocationGet(&takeOffLocation);

        if (takeOffLocation.Status == TAKEOFFLOCATION_STATUS_VALID) {
            PositionStateData positionState;
            PositionStateGet(&positionState);

            data.comp_gps.distance_to_home    = (uint16_t)sqrtf(Sq(takeOffLocation.East - positionState.East) + Sq(takeOffLocation.North - positionState.North));
            data.comp_gps.direction_to_home   = RAD2DEG(atan2f(takeOffLocation.East - positionState.East, takeOffLocation.North - positionState.North));

            data.comp_gps.home_position_valid = 1;
        }
    }

    // here, CC3D implementation could use raw gps data (GPSPositionSensorData) and locally cached GPSPositionSensorData at arming time as TakeOffLocation.


    ros_send(m, ROS_COMP_GPS, data.buf, sizeof(data));
}

static void ros_send_altitude(struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            int32_t estimatedAltitude; // cm
            int16_t vario; // cm/s
        } __attribute__((packed)) altitude;
    } data;


    if (PositionStateHandle() != NULL) {
        PositionStateData positionState;
        PositionStateGet(&positionState);

        data.altitude.estimatedAltitude = (int32_t)roundf(positionState.Down * -100.0f);
    } else {
        data.altitude.estimatedAltitude = 0;
    }

    if (VelocityStateHandle() != NULL) {
        VelocityStateData velocityState;
        VelocityStateGet(&velocityState);

        data.altitude.vario = (int16_t)roundf(velocityState.Down * -100.0f);
    } else {
        data.altitude.vario = 0;
    }

    ros_send(m, ROS_ALTITUDE, data.buf, sizeof(data));
}
#endif /* PIOS_EXCLUDE_ADVANCED_FEATURES */


// Scale stick values whose input range is -1 to 1 to ROS's expected
// PWM range of 1000-2000.
static uint16_t ros_scale_rc(float percent)
{
    return percent * 500 + 1500;
}

// Throttle maps to 1100-1900 and a bit differently as -1 == 1000 and
// then jumps immediately to 0 -> 1 for the rest of the range.  MWOSD
// can learn ranges as wide as they are sent, but defaults to
// 1100-1900 so the throttle indicator will vary for the same stick
// position unless we send it what it wants right away.
static uint16_t ros_scale_rc_thr(float percent)
{
    return percent <= 0 ? 1100 : percent * 800 + 1100;
}

// ROS RC order is Roll/Pitch/Yaw/Throttle/AUX1/AUX2/AUX3/AUX4
static void ros_send_channels(struct ros_bridge *m)
{
    AccessoryDesiredData acc0, acc1, acc2, acc3;
    ManualControlCommandData manualState;

    ManualControlCommandGet(&manualState);
    AccessoryDesiredInstGet(0, &acc0);
    AccessoryDesiredInstGet(1, &acc1);
    AccessoryDesiredInstGet(2, &acc2);
    AccessoryDesiredInstGet(3, &acc3);

    union {
        uint8_t  buf[0];
        uint16_t channels[8];
    } data = {
        .channels = {
            ros_scale_rc(manualState.Roll),
            ros_scale_rc(manualState.Pitch * -1),  // TL pitch is backwards
            ros_scale_rc(manualState.Yaw),
            ros_scale_rc_thr(manualState.Throttle),
            ros_scale_rc(acc0.AccessoryVal),
            ros_scale_rc(acc1.AccessoryVal),
            ros_scale_rc(acc2.AccessoryVal),
            ros_scale_rc(acc3.AccessoryVal),
        }
    };

    ros_send(m, ROS_RC, data.buf, sizeof(data));
}

static void ros_send_boxids(struct ros_bridge *m) // This is actually sending a map of ROS_STATUS.flag bits to BOX ids.
{
    ros_send(m, ROS_BOXIDS, ros_boxes, sizeof(ros_boxes));
}

static void ros_send_pidnames(struct ros_bridge *m)
{
    ros_send(m, ROS_PIDNAMES, (const uint8_t *)ros_pidnames, sizeof(ros_pidnames) - 1); // without terminating 0
}

static void pid_native2ros(const float *native, ros_pid_t *piditem, float scale, unsigned numelem)
{
    for (unsigned i = 0; i < numelem; ++i) {
        piditem->values[i] = lroundf(native[i] * scale);
    }
}

static void pid_ros2native(const ros_pid_t *piditem, float *native, float scale, unsigned numelem)
{
    for (unsigned i = 0; i < numelem; ++i) {
        native[i] = (float)piditem->values[i] / scale;
    }
}

static UAVObjHandle get_current_pid_bank_handle()
{
    uint8_t fm;

    ManualControlCommandFlightModeSwitchPositionGet(&fm);

    if (fm >= FLIGHTMODESETTINGS_FLIGHTMODEPOSITION_NUMELEM) {
        return STABILIZATIONSETTINGS_FLIGHTMODEMAP_BANK1;
    }

    StabilizationSettingsFlightModeMapOptions flightModeMap[STABILIZATIONSETTINGS_FLIGHTMODEMAP_NUMELEM];
    StabilizationSettingsFlightModeMapGet(flightModeMap);

    switch (flightModeMap[fm]) {
    case STABILIZATIONSETTINGS_FLIGHTMODEMAP_BANK1:
        return StabilizationSettingsBank1Handle();

    case STABILIZATIONSETTINGS_FLIGHTMODEMAP_BANK2:
        return StabilizationSettingsBank2Handle();

    case STABILIZATIONSETTINGS_FLIGHTMODEMAP_BANK3:
        return StabilizationSettingsBank3Handle();
    }

    return 0;
}

#define ALARM_OK    0
#define ALARM_WARN  1
#define ALARM_ERROR 2
#define ALARM_CRIT  3

#define MS2TICKS(m) ((m) / (portTICK_RATE_MS))

static void ros_send_alarms(__attribute__((unused)) struct ros_bridge *m)
{
    union {
        uint8_t buf[0];
        struct {
            uint8_t state;
            char    msg[MAX_ALARM_LEN];
        } __attribute__((packed)) alarm;
    } data;

    SystemAlarmsData alarm;
    SystemAlarmsGet(&alarm);

    // Special case early boot times -- just report boot reason

#if 0
    if (xTaskGetTickCount() < MS2TICKS(10 * 1000)) {
        data.alarm.state = ALARM_CRIT;
        const char *boot_reason = AlarmBootReason(alarm.RebootCause);
        strncpy((char *)data.alarm.msg, boot_reason, MAX_ALARM_LEN);
        data.alarm.msg[MAX_ALARM_LEN - 1] = '\0';
        ros_send(m, ROS_ALARMS, data.buf, strlen((char *)data.alarm.msg) + 1);
        return;
    }
#endif

    uint8_t state;
    data.alarm.state = ALARM_OK;
    int32_t len = AlarmString(&alarm, data.alarm.msg,
                              sizeof(data.alarm.msg), SYSTEMALARMS_ALARM_CRITICAL, &state); // Include only CRITICAL and ERROR

    // NOTE: LP alarm severity levels and ROS levels do not match. ERROR and CRITICAL are swapped.
    // So far, MW-OSD code (ROS consumer) does not make difference between ALARM_ERROR and ALARM_CRITICAL.
    // ALARM_WARN should be blinking if thats the highest severity level at the moment.
    // There might be other types of ROS consumers.

    switch (state) {
    case SYSTEMALARMS_ALARM_WARNING:
        data.alarm.state = ALARM_WARN;
        break;
    case SYSTEMALARMS_ALARM_ERROR:
        data.alarm.state = ALARM_CRIT;
        break;
    case SYSTEMALARMS_ALARM_CRITICAL:
        data.alarm.state = ALARM_ERROR;
        break;
    }

    ros_send(m, ROS_ALARMS, data.buf, len + 1);
}

static ros_state ros_state_checksum(struct ros_bridge *m, uint8_t b)
{
    if ((m->checksum ^ b) != 0) {
        return ROS_IDLE;
    }

    // Respond to interesting things.
    switch (m->cmd_id) {
    case ROS_RAW_GPS:
        ros_send_raw_gps(m);
        break;
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    case ROS_COMP_GPS:
        ros_send_comp_gps(m);
        break;
    case ROS_ALTITUDE:
        ros_send_altitude(m);
        break;
#endif /* PIOS_EXCLUDE_ADVANCED_FEATURES */
    case ROS_ATTITUDE:
        ros_send_attitude(m);
        break;
    case ROS_STATUS:
        ros_send_status(m);
        break;
    case ROS_ANALOG:
        ros_send_analog(m);
        break;
    case ROS_RC:
        ros_send_channels(m);
        break;
    case ROS_BOXIDS:
        ros_send_boxids(m);
        break;
    case ROS_ALARMS:
        ros_send_alarms(m);
        break;
    case ROS_PID:
        ros_send_pid(m);
        break;
    case ROS_SET_PID:
        ros_set_pid(m);
        break;
    case ROS_PIDNAMES:
        ros_send_pidnames(m);
        break;
    }

    return ROS_IDLE;
}

static ros_state ros_state_discard(struct ros_bridge *m, __attribute__((unused)) uint8_t b)
{
    return m->cmd_i++ == m->cmd_size ? ROS_IDLE : ROS_DISCARD;
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

#ifdef PIOS_EXCLUDE_ADVANCED_FEATURES
            ros->sensors |= ROS_SENSOR_ACC;
#else

            if (PIOS_SENSORS_GetInstanceByType(0, PIOS_SENSORS_TYPE_3AXIS_ACCEL)) {
                ros->sensors |= ROS_SENSOR_ACC;
            }
            if (PIOS_SENSORS_GetInstanceByType(0, PIOS_SENSORS_TYPE_1AXIS_BARO)) {
                ros->sensors |= ROS_SENSOR_BARO;
            }
            if (PIOS_SENSORS_GetInstanceByType(0, PIOS_SENSORS_TYPE_3AXIS_MAG | PIOS_SENSORS_TYPE_3AXIS_AUXMAG)) {
                ros->sensors |= ROS_SENSOR_MAG;
            }
#ifdef PIOS_SENSORS_TYPE_1AXIS_SONAR
            if (PIOS_SENSORS_GetInstanceByType(0, PIOS_SENSORS_TYPE_1AXIS_SONAR)) {
                ros->sensors |= ROS_SENSOR_SONAR;
            }
#endif /* PIOS_SENSORS_TYPE_1AXIS_SONAR */
#endif /* PIOS_EXCLUDE_ADVANCED_FEATURES */

            // MAP_SENSOR_GPS is hot-pluggable type

            HwSettingsADCRoutingDataArray adcRoutingDataArray;
            HwSettingsADCRoutingArrayGet(adcRoutingDataArray.array);

            for (unsigned i = 0; i < sizeof(adcRoutingDataArray.array) / sizeof(adcRoutingDataArray.array[0]); ++i) {
                switch (adcRoutingDataArray.array[i]) {
                case HWSETTINGS_ADCROUTING_BATTERYVOLTAGE:
                    ros->analog |= ROS_ANALOG_VOLTAGE;
                    break;
                case HWSETTINGS_ADCROUTING_BATTERYCURRENT:
                    ros->analog |= ROS_ANALOG_CURRENT;
                    break;
                default:;
                }
            }


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
