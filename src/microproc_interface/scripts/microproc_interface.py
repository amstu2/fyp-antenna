#!/usr/bin/env python
import rospy
import serial

from geometry_msgs.msg import Point32

elevation_limits_received = False
micro_serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)

def orientCallback(data):
    bearing = data.x
    elevation = data.y

def startNode():
    global elevation_limits_received
    micro_serial.write('B'.encode('utf-8'))
    while (not elevation_limits_received):
        line = micro_serial.readLine()
        rospy.loginfo(line)

    rospy.Subscriber('ant_orientation', Point32, orientCallback)
    rospy.init_node('micro_interface', anonymous=True)



if __name__ == '__main__':
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass