#!/usr/bin/env python
import rospy
import serial
import time

from geometry_msgs.msg import Point32

in_setup = True
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
    while(in_setup):
        line = micro_serial.readline()
        rospy.loginfo(line)
        if(line[0]=='L'):
            rospy.loginfo("Elevation limits received:")
            split_line = line.split('M')
            rospy.loginfo(split_line)




if __name__ == '__main__':
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
