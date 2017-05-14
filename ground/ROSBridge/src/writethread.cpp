/*
 ******************************************************************************
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       writethread.cpp
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

#include "rosbridge.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "uav_msgs/uav_pose.h"
#include <sstream>
#include "boost/thread.hpp"
#include "writethread.h"
#include "uavorosbridgemessage_priv.h"
#include "pios.h"


namespace librepilot {
class writethread_priv {
public:
    boost::thread *thread;
    ros::NodeHandle *nodehandle;
    rosbridge *parent;

    void poseCallback(const geometry_msgs::TransformStamped::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_pos_estimate_t *payload = (rosbridgemessage_pos_estimate_t *)message->data;

        // apply ENU to NED conversion - switch x/y, reverse z
        payload->position[0] = msg->transform.translation.y;
        payload->position[1] = msg->transform.translation.x;
        payload->position[2] = -msg->transform.translation.z;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_POS_ESTIMATE;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("received position, sending");
    }

    void commandCallback(const uav_msgs::uav_pose::ConstPtr & msg)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_flightcontrol_t *payload = (rosbridgemessage_flightcontrol_t *)message->data;

        payload->control[0] = msg->position.x;
        payload->control[1] = msg->position.y;
        payload->control[2] = msg->position.z;
        payload->control[3] = 0;
        payload->poi[0]     = msg->POI.x;
        payload->poi[1]     = msg->POI.y;
        payload->poi[2]     = msg->POI.z;
        payload->mode      = ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_WAYPOINT;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_FLIGHTCONTROL;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        parent->rosinfoPrint("control");
    }

    void run()
    {
        ros::Rate rate(0.1);

        ros::Subscriber subscriber1 = nodehandle->subscribe("vicon/octocopter/frame", 10, &writethread_priv::poseCallback, this);
        ros::Subscriber subscriber2 = nodehandle->subscribe(parent->getNameSpace() + "/command", 10, &writethread_priv::commandCallback, this);

        while (ros::ok()) {
            uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
            rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
            rosbridgemessage_pingpong_t *payload = (rosbridgemessage_pingpong_t *)message->data;
            payload->sequence_number = parent->getMySequenceNumber() + 1;
            parent->setMySequenceNumber(payload->sequence_number);
            message->magic     = ROSBRIDGEMAGIC;
            message->type      = ROSBRIDGEMESSAGE_PING;
            message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
            boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - *parent->getStart();
            message->timestamp = diff.total_microseconds();
            message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
            int res = parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
            std_msgs::String msg;
            std::stringstream bla;
            bla << "sending a ping myself " << (unsigned int)payload->sequence_number;
            bla << " write res is  : " << (int)res;
            msg.data = bla.str();
            parent->rosinfoPrint(msg.data.c_str());
            rate.sleep();
        }
    }
};

writethread::writethread(ros::NodeHandle *nodehandle, rosbridge *parent)
{
    instance = new writethread_priv();
    instance->parent     = parent;
    instance->nodehandle = nodehandle;
    instance->thread     = new boost::thread(boost::bind(&writethread_priv::run, instance));
}

writethread::~writethread()
{
    instance->thread->detach();
    delete instance->thread;
    delete instance;
}
}
