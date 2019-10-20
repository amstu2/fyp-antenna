#!/usr/bin/env python
import rospy
import serial
import time

from geometry_msgs.msg import Point32

in_setup = True
min_elevation = 20
max_elevation = -9
micro_serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)

def orientCallback(data):
    bearing = data.x
    elevation = data.y

def startNode():
    global min_elevation
    global max_elevation
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
            min_elevation = int(split_line[1])/100.0
            rospy.loginfo(min_elevation)
            max_elevation = int(split_line[2])/100.0
            rospy.loginfo(max_elevation)
            in_setup = False




if __name__ == '__main__':
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
