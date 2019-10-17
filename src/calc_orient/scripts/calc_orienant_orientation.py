#!/usr/bin/env python

import rospy
import math
from  sensor_msgs.msg import NavSatFix


class Entity:
    def __init__(self, name = 'Untitled'):
        self.name = name
        self.latitude = 0.00000
        self.longitude = 0.00000
        self.altitude = 0.000
        self.has_GPS_fix = False

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
        return bearing

    def getDistanceToEntity(self, external_entity):
        EARTH_RADIUS = 6371
        delta_x = (math.radians(external_entity.longitude) - math.radians(self.longitude)) * math.cos((math.radians(self.latitude) + math.radians(external_entity.latitude))/2)
        delta_y = (math.radians(external_entity.latitude) - math.radians(self.latitude))
        return (EARTH_RADIUS * math.sqrt(pow(delta_x,2)+pow(delta_y,2)))


    def getElevationToEntity(self, external_entity):
        rospy.loginfo(getDistanceToEntity(self,external_entity))
        
        

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
    antenna.has_GPS_fix = True
    antenna.ROSLogGPSCoordinates()


def roverGPSCallback(data):
    rover.setGPSCoordinates(data.latitude, data.longitude)
    rover.ROSLogGPSCoordinates()
    if(antenna.has_GPS_fix):
        rospy.loginfo(antenna.getElevationToEntity(rover))

def calculateAntennaOrientation():
    rospy.Subscriber('ant_gps', NavSatFix, antGPSCallback)
    rospy.Subscriber('rover_gps', NavSatFix, roverGPSCallback)
    rospy.init_node('ant_orient', anonymous=True)
    rospy.spin()


antenna = Entity('Antenna')
rover = Entity('Rover')

if __name__ == '__main__':
    try:
        calculateAntennaOrientation()
    except rospy.ROSInterruptException:
        pass
