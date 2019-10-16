#!/usr/bin/env python

#import rospy
import math
#from  sensor_msgs.msg import NavSatFix


class Entity:
    def __init__(self, name = 'Untitled', latitude = 0.00, longitude = 0.00):
        self.name = name
        self.latitude = latitude
        self.longitude = longitude

    def ROSLogGPSCoordinates(self):
        rospy.loginfo("\r\n" + self.name + " GPS Coordinates: \r\nLatitude: " + str(self.latitude) + "\r\nLongitude: " + str(self.longitude))
    
    def printGPSCoordinates(self):
        print("\r\n" + self.name + " GPS Coordinates: \r\nLatitude: " + str(self.latitude) + "\r\nLongitude: " + str(self.longitude))

    def setGPSCoordinates(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
        
    def getBearingToEntity(self, external_entity):
        y = math.sin(external_entity.longitude - self.longitude) * math.cos(external_entity.latitude)
        x = math.cos(self.latitude)*math.sin(external_entity.latitude) - math.sin(self.latitude)*math.cos(external_entity.latitude)*math.cos(external_entity.longitude-self.longitude)
        bearing = math.degrees(math.atan2(y, x))
        return bearing

    def getBearingToEntity2(self, extern_entity_lat, extern_entity_long):
        y = math.sin(extern_entity_long - self.longitude) * math.cos(extern_entity_lat)
        x = math.cos(self.latitude)*math.sin(extern_entity_lat) - math.sin(self.latitude)*math.cos(extern_entity_lat)*math.cos(extern_entity_long-self.longitude)
        bearing = math.degrees(math.atan2(y, x))
        return bearing


        
def calculateAntennaBearing():
    antenna.getBearingToEntity(rover)

def antGPSCallback(data):
    rover.latitude = data.latitude
    rover.longitude = data.longitude


def roverGPSCallback(data):
    rover.latitude = data.latitude
    rover.longitude = data.longitude

def calculateAntennaOrientation():
    rospy.Subscriber('ant_gps', NavSatFix, antGPSCallback)
    rospy.Subscriber('rover_gps', NavSatFix, roverGPSCallback)
    rospy.init_node('ant_orient', anonymous=True)
    rospy.spin()

antenna = Entity('Antenna',-37.937804, 145.013067)
rover = Entity('Rover',-37.936482, 145.014253)

if __name__ == '__main__':
    try:
        calculateAntennaOrientation()
    except rospy.ROSInterruptException:
        pass