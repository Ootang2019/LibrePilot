#!/usr/bin/env python2

import rospy
import socket
from geometry_msgs.msg import PoseStamped

server = ("127.0.0.1",40000)
topic = ('ROSGCS2',PoseStamped)

rospy.init_node('GCS2ROS')
rospy.loginfo("Startup")

UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPSocket.bind(server)
msgpublisher = rospy.Publisher(topic[0],topic[1],queue_size=3)
rospy.loginfo("socket bound, publisher initialized")

def NetworkHandler(values):

    #12 actuator channels, not much we can do with that for now
    msg=PoseStamped()
    msgpublisher.publish(msg)
    rospy.loginfo("msg sent to ros")
    return

while True:
    data,sender = UDPSocket.recvfrom(4096)
    values = [ float(v.strip()) for v in data.split(',')]
    NetworkHandler(values)








