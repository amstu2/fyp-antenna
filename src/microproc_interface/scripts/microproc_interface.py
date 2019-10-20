#!/usr/bin/env python
import rospy
import serial
import time

from geometry_msgs.msg import Point32

elevation_limits_received = False
micro_serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)

def orientCallback(data):
    bearing = data.x
    elevation = data.y

def startNode():
    global elevation_limits_received
    rospy.Subscriber('ant_orientation', Point32, orientCallback)
    rospy.init_node('micro_interface', anonymous=True)
    time.sleep(1)
    micro_serial.write('B'.encode('utf-8'))
    rospy.loginfo('B written')
    while (1):
        line = micro_serial.readline()
        rospy.loginfo(line)
        #time.sleep(1)
        #micro_serial.write('B'.encode('utf-8'))
        #rospy.loginfo('B 2')




if __name__ == '__main__':
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
