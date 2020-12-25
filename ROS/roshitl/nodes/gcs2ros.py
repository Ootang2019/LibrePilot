#!/usr/bin/env python2

import rospy
import socket
from std_msgs.msg import MultiArrayDimension
from librepilot.msg import LibrepilotActuators

#from geometry_msgs.msg import PoseStamped
sequence=0

server = ("127.0.0.1",40000)
topic = ('GCSACTUATORS',LibrepilotActuators)

rospy.init_node('GCS2ROS')
rospy.loginfo("Startup")

UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPSocket.bind(server)
msgpublisher = rospy.Publisher(topic[0],topic[1],queue_size=3)
rospy.loginfo("socket bound, publisher initialized")


def NetworkHandler(values):

    #12 actuator channels, send as multiarray
    global sequence

    msg=LibrepilotActuators()
    msg.header.frame_id="world"
    msg.header.seq = sequence
    sequence+=1
    msg.header.stamp=rospy.Time.now()
    msg.data.layout.dim=[MultiArrayDimension()]
    msg.data.layout.dim[0].label="Channel"
    msg.data.layout.dim[0].size=12
    msg.data.layout.dim[0].stride=1
    msg.data.data=values
    msgpublisher.publish(msg)
    rospy.loginfo("msg sent to ros")
    return

while True:
    data,sender = UDPSocket.recvfrom(4096)
    values = [ float(v.strip()) for v in data.split(',')]
    NetworkHandler(values)








