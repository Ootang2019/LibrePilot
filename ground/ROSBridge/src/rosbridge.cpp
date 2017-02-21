/*
 ******************************************************************************
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       rosbridge.cpp
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
#include "ros/ros.h"
#include "boost/asio.hpp"
#include <boost/lexical_cast.hpp>
#include "readthread.h"
#include "writethread.h"
#include <string>

namespace librepilot {
rosbridge *globalRosbridge;

/*
        void sigHandler(const boost::system::error_code& error, int signal_number) {
                // shutdown read and write threads too
                exit(-1);
                ros::shutdown();
        }
 */

class rosbridge_priv {
public:
    int argc;
    char * *argv;
    ros::NodeHandle *nodehandle = NULL;
    uint8_t mySequenceNumber;
    boost::asio::serial_port *revolution;
    boost::asio::io_service io_service;
    boost::posix_time::ptime start;
    boost::mutex serial_Mutex;
    boost::mutex ROSinfo_Mutex;
    std::string nameSpace;
    // ...
};

rosbridge::rosbridge(int argc, char * *argv)
{
    globalRosbridge = this;
    instance = new rosbridge_priv();
    instance->argc  = argc;
    instance->argv  = argv;
    instance->start = boost::posix_time::microsec_clock::local_time();
}

rosbridge::~rosbridge()
{
    if (instance->nodehandle) {
        delete instance->nodehandle;
    }
    delete instance;
}

int rosbridge::run(void)
{
    ros::init(instance->argc, instance->argv, "librepilot");

    instance->nodehandle = new ros::NodeHandle();
    // boost::asio::signal_set signals(instance->io_service, SIGINT, SIGTERM);
    // signals.async_wait(sigHandler);

    if (instance->argc < 4) {
        printf("Usage: %s <namespace> <serial_port> <baudrate>\n", instance->argv[0]);
        return -1;
    }

    instance->nameSpace = std::string(instance->argv[1]);

    // open tty device
    boost::asio::serial_port revolution(instance->io_service);
    instance->revolution = &revolution;
    revolution.open(instance->argv[2]);
    revolution.set_option(boost::asio::serial_port_base::baud_rate(boost::lexical_cast<int>(instance->argv[3])));

    readthread reader(instance->nodehandle, &revolution, this);
    writethread writer(instance->nodehandle, this);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    // join the other threads
    return 0;
}

boost::posix_time::ptime *rosbridge::getStart(void)
{
    return &instance->start;
}

void rosbridge::setMySequenceNumber(uint8_t value)
{
    instance->mySequenceNumber = value;
}
uint8_t rosbridge::getMySequenceNumber(void)
{
    return instance->mySequenceNumber;
}

int rosbridge::serialWrite(uint8_t *buffer, size_t length)
{
    instance->serial_Mutex.lock();
    int res = boost::asio::write(*instance->revolution, boost::asio::buffer(buffer, length));
    instance->serial_Mutex.unlock();


    return res;
}

void rosbridge::rosinfoPrint(const char *bla)
{
    instance->ROSinfo_Mutex.lock();
    ROS_INFO("%s", bla);
    instance->ROSinfo_Mutex.unlock();
}
}
