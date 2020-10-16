#!/usr/bin/env python2

import rospy
import socket
from geometry_msgs.msg import PoseStamped

server = ("127.0.0.1",40001)
topic = ('ROSGCS',PoseStamped)

UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)



def MessageSubscriber(msg):

    #rotation[x,y,z]
    #accel[x,y,z]
    #orientation[R,P,Y]
    #airspeed
    #velocity[N,E,D]
    #position[N,E,D]
    #temperature
    #pressure
    string="0,0,0,0,0,9.81,0,0,0,0,0,0,0,0,0,0,0,-1\n"
    UDPSocket.sendto(str.encode(string),server)
    rospy.loginfo("msg sent to gcs")
    return



rospy.init_node('ROS2GCS')
rospy.loginfo("Startup")
msgsubscriber = rospy.Subscriber(topic[0],topic[1],MessageSubscriber)
rospy.loginfo("Subscriber started")

rospy.spin()





