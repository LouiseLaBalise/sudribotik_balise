#!/usr/bin/env python3

import rospy
from balise.msg import PlantCoordinates
import random

class PlantPublisher:

    def __init__(self):
        rospy.init_node('plant_publisher', anonymous=True)
        self.publisher = rospy.Publisher('plant_coordinates', PlantCoordinates, queue_size=10)
        rospy.Timer(rospy.Duration(5.0), self.timer_callback)

    def timer_callback(self, event):
        plant_msg = PlantCoordinates()
        plant_msg.id = random.randint(1, 36)  # Replace with your actual ID range
        plant_msg.x = random.uniform(0, 100)
        plant_msg.y = random.uniform(0, 100)
        plant_msg.theta = random.uniform(0, 360)

        rospy.loginfo(f"Publishing plant: ID {plant_msg.id}, Coordinates: {plant_msg.x}, {plant_msg.y}, {plant_msg.theta}")
        self.publisher.publish(plant_msg)

def main():
    plant_publisher = PlantPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
