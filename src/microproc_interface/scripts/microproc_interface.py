#!/usr/bin/env python
import rospy
import serial
import time

from geometry_msgs.msg import Point32

in_setup = True
min_elevation = 20 # arbitrary
max_elevation = -9
micro_serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)

def orientCallback(data):
    if(in_setup == False):
        raw_bearing = round(data.x,1)
        raw_elevation = round(data.y,1)
        bearing = raw_bearing * 10
        elevation = raw_elevation * 10
        rospy.loginfo(raw_bearing)
        rospy.loginfo(raw_elevation)
        bearing_hund = str(bearing // 1000)
        bearing = bearing % 1000
        bearing_ten = str(bearing // 100)
        bearing = bearing % 100
        bearing_one = str(bearing // 10)
        bearing = bearing % 10
        bearing_dec = str(bearing // 1)
        if(raw_elevation < 0):
            elevation_sign = '-'
        else:
            elevation_sign = '+'
        elevation_ten = str(elevation // 100)
        elevation = elevation % 100
        elevation_one = str(elevation // 10)
        elevation = elevation % 10
        elevation_dec = str(bearing // 1)

        rospy.loginfo(bearing_hund+bearing_ten+bearing_one+bearing_dec+elevation_sign+elevation_ten+elevation_one+elevation_dec)

def startNode():
    global in_setup
    global min_elevation
    global max_elevation
    pos_msg = Point32()
    pos_pub = rospy.Publisher('current_orient', Point32, queue_size=10) 
    rospy.Subscriber('ant_orientation', Point32, orientCallback)
    rospy.init_node('micro_interface', anonymous=True)
    time.sleep(1)
    micro_serial.write('B'.encode('utf-8'))
    rospy.loginfo('B written')
    while(in_setup):
        line = micro_serial.readline()
        rospy.loginfo(line)
        if(line[0]=='L'):
            split_line = line.split('M')
            rospy.loginfo(split_line)
            min_elevation = int(split_line[1])/100.0
            rospy.loginfo("Minimum Elevation: " + str(min_elevation))
            max_elevation = int(split_line[2])/100.0
            rospy.loginfo("Maximum Elevation: " + str(max_elevation))
            in_setup = False
    serial_open = True
    while(serial_open):
        line = micro_serial.readline()
        if(line[0] == 'O'):
            e_split = line.split('E')
            a_split = e_split[0].split('A')
            current_azimuth = int(a_split[1])
            current_elevation = int(e_split[1])/100.0
            pos_msg.x = current_azimuth
            pos_msg.y = current_elevation
            pos_pub.publish(pos_msg)
            rospy.loginfo(current_azimuth)
            rospy.loginfo(current_elevation)





if __name__ == '__main__':
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
