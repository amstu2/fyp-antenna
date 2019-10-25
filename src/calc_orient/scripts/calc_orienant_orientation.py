#!/usr/bin/env python

import rospy
import math
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point32
from std_msgs.msg import Int16
from std_msgs.msg import Bool

antenna_is_auto = True
radio_tx_rate = 50
radio_signal_strength = -96

orient_pub = rospy.Publisher('ant_orientation', Point32, queue_size=10)
msg = Point32()
sweeping_enabled = False

class Entity:
    def __init__(self, name = 'Untitled'):
        self.name = name
        self.latitude = 0.00000
        self.longitude = 0.00000
        self.altitude = 0.000
        self.has_GPS_fix = False
        self.azimuth = 0.0
        self.elevation = 0.0
        self.radio_tx_rate = 0.0

    def ROSLogGPSCoordinates(self):
        rospy.loginfo("\r\n" + self.name + " GPS Coordinates: \r\nLatitude: " + str(self.latitude) + "\r\nLongitude: " + str(self.longitude))
    
    def printGPSCoordinates(self):
        print("\r\n" + self.name + " GPS Coordinates: \r\nLatitude: " + str(self.latitude) + "\r\nLongitude: " + str(self.longitude))

    def setGPSCoordinates(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def setAltitude(self, altitude):
        self.altitude = altitude
        
    def getBearingToEntity(self, external_entity):
        y = math.sin(external_entity.longitude - self.longitude) * math.cos(external_entity.latitude)
        x = math.cos(self.latitude)*math.sin(external_entity.latitude) - math.sin(self.latitude)*math.cos(external_entity.latitude)*math.cos(external_entity.longitude-self.longitude)
        bearing = math.degrees(math.atan2(y, x))
        if(bearing < 0):
            bearing = 360 + bearing
        rospy.loginfo("Bearing: " + str(bearing))
        return bearing


    def getElevationToEntity(self, external_entity):
        distance = self.getDistanceToEntity(external_entity)
        altitude_difference = external_entity.altitude - self.altitude
        elevation_radians = math.atan2(altitude_difference, distance)
        elevation_degrees = math.degrees(elevation_radians)
        rospy.loginfo("Elevation: " + str(elevation_degrees))
        return elevation_degrees

    def getDistanceToEntity(self, external_entity):
        EARTH_RADIUS = 6371
        x = (math.radians(external_entity.longitude) - math.radians(self.longitude)) * math.cos((math.radians(self.latitude) + math.radians(external_entity.latitude))/2)
        y = (math.radians(external_entity.latitude) - math.radians(self.latitude))
        distance = EARTH_RADIUS * math.sqrt(pow(x,2)+pow(y,2))
        distance_metres = distance * 1000
        rospy.loginfo("Distance: " + str(distance_metres))
        return distance_metres
        
    def getBearingToEntity2(self, extern_entity_lat, extern_entity_long):
        y = math.sin(extern_entity_long - self.longitude) * math.cos(extern_entity_lat)
        x = math.cos(self.latitude)*math.sin(extern_entity_lat) - math.sin(self.latitude)*math.cos(extern_entity_lat)*math.cos(extern_entity_long-self.longitude)
        bearing = math.degrees(math.atan2(y, x))
        if(bearing < 0):
            bearing = 360 + bearing
        return bearing


        
def calculateAntennaBearing():
    return antenna.getBearingToEntity(rover)

def antGPSCallback(data):
    antenna.setGPSCoordinates(data.latitude, data.longitude)
    antenna.setAltitude(data.altitude)
    antenna.has_GPS_fix = True
    antenna.ROSLogGPSCoordinates()


def roverGPSCallback(data):
    global antenna_is_auto
    rover.setGPSCoordinates(data.latitude, data.longitude)
    rover.setAltitude(data.altitude)
    rover.ROSLogGPSCoordinates()
    rover.has_GPS_fix = True
    if(antenna.has_GPS_fix and antenna_is_auto and (sweeping_enabled == False)):
        raw_bearing = antenna.getBearingToEntity(rover)
        raw_elevation = antenna.getElevationToEntity(rover)
        msg.x = raw_bearing
        msg.y = raw_elevation
        orient_pub.publish(msg)

def autoSwitchCallback(data):
    global antenna_is_auto 
    antenna_is_auto = data.data

def sweepCallback(data):
    global sweeping_enabled
    global antenna_is_auto
    global radio_tx_rate
    sweeping_enabled = data
def radioTXCallback(data):
    global radio_tx_rate
    radio_tx_rate = data.data

def radioSignalCallback(data):
    global radio_signal_strength
    radio_signal_strength = data.data

            


            
def currentOrientationCallback(data):
    antenna.azimuth = data.x
    antenna.elevation = data.y
def calculateAntennaOrientation():
    rospy.Subscriber('ant_gps', NavSatFix, antGPSCallback)
    rospy.Subscriber('rover_gps', NavSatFix, roverGPSCallback)
    rospy.Subscriber('ant_is_auto', Bool, autoSwitchCallback)
    rospy.Subscriber('current_orient',Point32, currentOrientationCallback)
    rospy.Subscriber('sweep_enabled', Bool, sweepCallback)
    rospy.Subscriber('/radio_signal', Int16, radioSignalCallback)
    rospy.Subscriber('/radio_tx_rate',Int16, radioTXCallback)
    rospy.init_node('ant_orient', anonymous=True)
    rospy.spin()


antenna = Entity('Antenna')
rover = Entity('Rover')

if __name__ == '__main__':
    try:
        calculateAntennaOrientation()
    except rospy.ROSInterruptException:
        pass
