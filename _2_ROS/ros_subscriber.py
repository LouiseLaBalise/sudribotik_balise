# ros_subscribers.py

import rospy
from std_msgs.msg import String  # Import the appropriate message type for your data
import sqlite3

def callback(data):
    # Split received data into ID and coordinates
    data_parts = data.data.split(',')
    
    if len(data_parts) >= 4:  # Ensure at least one coordinate and an ID are present
        object_id = int(data_parts[0])
        coordinates = list(map(float, data_parts[1:]))  # Extract coordinates as floats

        # Process the received data (you can customize this part)
        print(f"Received data for object ID {object_id}: {coordinates}")

        # Save data to the SQL database
        save_to_database(object_id, coordinates)

def save_to_database(object_id, coordinates):
    conn = sqlite3.connect('your_database.db')  # Replace 'your_database.db' with your actual database name
    cursor = conn.cursor()

    cursor.execute('''
        INSERT INTO object_coordinates (id, x, y, z)
        VALUES (?, ?, ?, ?)
    ''', (object_id, coordinates[0], coordinates[1], coordinates[2]))

    conn.commit()
    conn.close()

def subscriber():
    rospy.init_node('object_coordinates_subscriber', anonymous=True)
    rospy.Subscriber('object_coordinates_topic', String, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()
