# ros_publishers.py

import rospy
from std_msgs.msg import String  # Import the appropriate message type for your data

def publish_coordinates(coordinates, topic, object_id):
    rospy.init_node(f'{topic}_publisher', anonymous=True)
    pub = rospy.Publisher(f'{topic}_coordinates_topic', String, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Add object ID to coordinates
        coordinates_with_id = [object_id] + coordinates
        coordinates_str = ",".join(map(str, coordinates_with_id))
        
        rospy.loginfo(coordinates_str)
        pub.publish(coordinates_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example coordinates, replace with your actual data
        robot1_coordinates = [1.23, 4.56, 7.89]
        aruco_coordinates = [2.34, 5.67, 8.90]

        # Coordinates for specific quantities
        plant_coordinates = [1.0, 2.0, 3.0]  # Replace with actual plant coordinates
        pami_coordinates = [4.0, 5.0, 6.0]  # Replace with actual pami coordinates
        solarp_coordinates = [7.0, 8.0, 9.0]  # Replace with actual solarp coordinates

        # Publish coordinates for each category and quantity with ID
        for plant_id in range(1, 37):  # 36 plants
            publish_coordinates(plant_coordinates, 'plants', plant_id)

        for pami_id in range(1, 7):  # 6 pami
            publish_coordinates(pami_coordinates, 'pami', pami_id)

        for robot_id in range(1, 3):  # 2 robots
            publish_coordinates(robot1_coordinates, 'robot1', robot_id)

        for solarp_id in range(1, 10):  # 9 solarp
            publish_coordinates(solarp_coordinates, 'solarp', solarp_id)

        # You can add more categories and quantities as needed

    except rospy.ROSInterruptException:
        pass
