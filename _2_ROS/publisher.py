<<<<<<< Updated upstream
import rospy
from std_msgs.msg import String  # Import the appropriate message type for your data

def publish_coordinates(coordinates):
    rospy.init_node('object_coordinates_publisher', anonymous=True)
    pub = rospy.Publisher('object_coordinates_topic', String, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        coordinates_str = ",".join(map(str, coordinates))
        rospy.loginfo(coordinates_str)
        pub.publish(coordinates_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example coordinates, replace with your actual data
        object_coordinates = [1.23, 4.56, 7.89]
        publish_coordinates(object_coordinates)
    except rospy.ROSInterruptException:
=======
import rospy
from std_msgs.msg import String  # Import the appropriate message type for your data

def publish_coordinates(coordinates):
    rospy.init_node('object_coordinates_publisher', anonymous=True)
    pub = rospy.Publisher('object_coordinates_topic', String, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        coordinates_str = ",".join(map(str, coordinates))
        rospy.loginfo(coordinates_str)
        pub.publish(coordinates_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example coordinates, replace with your actual data
        object_coordinates = [1.23, 4.56, 7.89]
        publish_coordinates(object_coordinates)
    except rospy.ROSInterruptException:
>>>>>>> Stashed changes
        pass