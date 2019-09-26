#!/usr/bin/env python

import rospy
import serial
from sensor_msgs.msg import NavSatFix

ROS_REFRESH_RATE = 1

class SerialInterface:
    def __init__(self, serial_channel, baud, timeout):
        self.received_data = ""
        self.serial_address = serial_channel
        self.baud_rate = baud
        self.timeout = timeout
        self.uart = serial.Serial(self.serial_address, baudrate=self.baud_rate, timeout=self.timeout)
        

    def clearReceivedData(self):
        self.received_data = ""

    def readSerialInput(self):
        self.clearReceivedData()
        while(self.uart.inWaiting() > 0):
            self.received_data += self.uart.read()
        return self.received_data

class NMEAParser:
    def __init__(self, NMEA_type):
        self.__NMEA_ID = -1
        self.NMEA_version = NMEA_type
        if(NMEA_type == 'NMEA_0183'):
            self.__NMEA_ID = 0

    def getNMEAVersion(self):
        return self.__NMEA_ID

    def __convertDMToDD(self, deg, min, direction):
        direction_modifier = 1.0
        if((direction == "W") or (direction == "S")):
            direction_modifier = -1.0
        return (direction_modifier*(deg + (min/60.0)))

    def __formatGPSData(self, data):
        if(self.__NMEA_ID == 0):
            raw_latitude = data[0]
            latitude_direction = data[1]
            raw_longitude = data[2]
            longitude_direction = data[3]

            lat_deg = int(raw_latitude[0:2])
            lat_min = float(raw_latitude[2:])
            long_deg = int(raw_longitude[0:3])
            long_min = float(raw_longitude[3:])

            latitude_DD = self.__convertDMToDD(lat_deg, lat_min, latitude_direction)
            longitude_DD = self.__convertDMToDD(long_deg, long_min, longitude_direction)

            return [latitude_DD, longitude_DD]

    def getGPSLocation(self, data):
        if(self.__NMEA_ID == 0):
            split_lines = []
            split_lines = data.split('$')
            #rospy.loginfo(split_lines)
            for data_line in split_lines:
                if(data_line[2:5] == 'RMC'):
                    split_RMC_line = data_line.split(',')
                    if(len(split_RMC_line) <> 13):
                        rospy.logwarn("INVALID RMC READING - CHECK FOR HARDWARE CORRUPTION")
                        return
                    rospy.loginfo(split_RMC_line)
                    if(split_RMC_line[2] == 'V'):
                        rospy.logwarn("NO GPS FIX")
                        return
                    elif(split_RMC_line[2] == 'A'):
                        rospy.loginfo("Found GPS")
                        return self.__formatGPSData(split_RMC_line[3:7])
                    else:
                        rospy.logwarn("INVALID RMC READING - CHECK FOR HARDWARE CORRUPTION")
                        return



def transmitGPS():
    pub = rospy.Publisher('ant_gps', NavSatFix, queue_size=10)
    rospy.init_node('antenna_gps', anonymous=True)
    rate = rospy.Rate(ROS_REFRESH_RATE)
    parser = NMEAParser('NMEA_0183')
    while not rospy.is_shutdown():
        received_data = gps_interface.readSerialInput()
        [latitude, longitude] = parser.getGPSLocation(received_data)
        pub.latitude = latitude
        pub.longitude = longitude
        rate.sleep()

if __name__ == '__main__':
    gps_interface = SerialInterface("/dev/serial0", 9600, 3000)
    try:
        transmitGPS()
    except rospy.ROSInterruptException:
        pass
