/*
 ******************************************************************************
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       readthread.cpp
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
#include <sstream>
#include "boost/thread.hpp"
#include "readthread.h"
#include "uavorosbridgemessage_priv.h"
#include "pios.h"

namespace librepilot {
class readthread_priv {
public:
    boost::asio::serial_port *port;
    boost::thread *thread;
    ros::NodeHandle *nodehandle;
    uint8_t rx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
    size_t rx_length;
    rosbridge *parent;


/**
 * Process incoming bytes from an ROS query thing.
 * @param[in] b received byte
 * @return true if we should continue processing bytes
 */
    void ros_receive_byte(uint8_t b)
    {
        rx_buffer[rx_length] = b;
        rx_length++;
        rosbridgemessage_t *message = (rosbridgemessage_t *)rx_buffer;

        // very simple parser - but not a state machine, just a few checks
        if (rx_length <= offsetof(rosbridgemessage_t, length)) {
            // check (partial) magic number - partial is important since we need to restart at any time if garbage is received
            uint32_t canary = 0xff;
            for (uint32_t t = 1; t < rx_length; t++) {
                canary = (canary << 8) || 0xff;
            }
            if ((message->magic & canary) != (ROSBRIDGEMAGIC & canary)) {
                // parse error, not beginning of message
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "canary failure";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
        }
        if (rx_length == offsetof(rosbridgemessage_t, timestamp)) {
            if (message->length > (uint32_t)(ROSBRIDGEMESSAGE_BUFFERSIZE - offsetof(rosbridgemessage_t, data))) {
                // parse error, no messages are that long
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "zero length";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
        }
        if (rx_length == offsetof(rosbridgemessage_t, crc32)) {
            if (message->type >= (uint32_t)ROSBRIDGEMESSAGE_END_ARRAY_SIZE) {
                // parse error
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "invalid type";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
            if (message->length != ROSBRIDGEMESSAGE_SIZES[message->type]) {
                // parse error
                rx_length = 0;
                {
                    std_msgs::String msg;
                    std::stringstream bla;
                    bla << "invalid length";
                    msg.data = bla.str();
                    parent->rosinfoPrint(msg.data.c_str());
                }
                return;
            }
        }
        if (rx_length < offsetof(rosbridgemessage_t, data)) {
            // not a parse failure, just not there yet
            return;
        }
        if (rx_length == offsetof(rosbridgemessage_t, data) + ROSBRIDGEMESSAGE_SIZES[message->type]) {
            // complete message received and stored in pointer "message"
            // empty buffer for next message
            rx_length = 0;

            if (PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length) != message->crc32) {
                std_msgs::String msg;
                std::stringstream bla;
                bla << "CRC mismatch";
                msg.data = bla.str();
                parent->rosinfoPrint(msg.data.c_str());
                // crc mismatch
                return;
            }
            switch (message->type) {
            case ROSBRIDGEMESSAGE_PING:
                pong_handler((rosbridgemessage_pingpong_t *)message->data);
                break;
            default:
            {
                std_msgs::String msg;
                std::stringstream bla;
                bla << "received something";
                msg.data = bla.str();
                parent->rosinfoPrint(msg.data.c_str());
            }
                // do nothing at all and discard the message
            break;
            }
        }
    }


    void pong_handler(rosbridgemessage_pingpong_t *data)
    {
        uint8_t tx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
        rosbridgemessage_t *message = (rosbridgemessage_t *)tx_buffer;
        rosbridgemessage_pingpong_t *payload = (rosbridgemessage_pingpong_t *)message->data;

        *payload = *data;
        message->magic     = ROSBRIDGEMAGIC;
        message->type      = ROSBRIDGEMESSAGE_PONG;
        message->length    = ROSBRIDGEMESSAGE_SIZES[message->type];
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - *parent->getStart();
        message->timestamp = diff.total_microseconds();
        message->crc32     = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
        int res = parent->serialWrite(tx_buffer, message->length + offsetof(rosbridgemessage_t, data));
        std_msgs::String msg;
        std::stringstream bla;
        bla << "received ping : " << (unsigned int)payload->sequence_number;
        bla << " pong write res is  : " << (int)res;
        msg.data = bla.str();
        parent->rosinfoPrint(msg.data.c_str());
        // chatter_pub.publish(msg);
    }


    void run()
    {
        unsigned char c;
        ros::Publisher chatter_pub = nodehandle->advertise<std_msgs::String>("chatter", 1000);

        rx_length = 0;
        while (ros::ok()) {
            boost::asio::read(*port, boost::asio::buffer(&c, 1));
            ros_receive_byte(c);
            std_msgs::String msg;
            std::stringstream bla;
            bla << std::hex << (unsigned short)c;
            msg.data = bla.str();
            parent->rosinfoPrint(msg.data.c_str());
            // chatter_pub.publish(msg);
        }
    }
};

readthread::readthread(ros::NodeHandle *nodehandle, boost::asio::serial_port *port, rosbridge *parent)
{
    instance = new readthread_priv();
    instance->parent     = parent;
    instance->port       = port;
    instance->nodehandle = nodehandle;
    instance->thread     = new boost::thread(boost::bind(&readthread_priv::run, instance));
}

readthread::~readthread()
{
    instance->thread->detach();
    delete instance->thread;
    delete instance;
}
}
