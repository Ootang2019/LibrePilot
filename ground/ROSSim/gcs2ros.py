#!/usr/bin/env python2

import rospy
import socket
from std_msgs.msg import Float64MultiArray,MultiArrayDimension

#from geometry_msgs.msg import PoseStamped

server = ("127.0.0.1",40000)
topic = ('GCSACTUATORS',Float64MultiArray)

rospy.init_node('GCS2ROS')
rospy.loginfo("Startup")

UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPSocket.bind(server)
msgpublisher = rospy.Publisher(topic[0],topic[1],queue_size=3)
rospy.loginfo("socket bound, publisher initialized")

def NetworkHandler(values):

    #12 actuator channels, send as multiarray

    msg=Float64MultiArray()
    msg.layout.dim=[MultiArrayDimension()]
    msg.layout.dim[0].label="Channel"
    msg.layout.dim[0].size=12
    msg.layout.dim[0].stride=1
    msg.data=values
    msgpublisher.publish(msg)
    rospy.loginfo("msg sent to ros")
    return

while True:
    data,sender = UDPSocket.recvfrom(4096)
    values = [ float(v.strip()) for v in data.split(',')]
    NetworkHandler(values)








