#!/usr/bin/env python3

import rospy
from balise.msg import RobotCoordinates, PlantCoordinates
import sqlite3
import cv2
import numpy as np

class RobotPublisher:

    def __init__(self):
        rospy.init_node('robot_publisher', anonymous=True)
        self.publisher = rospy.Publisher('robot_coordinates', RobotCoordinates, queue_size=10)
        rospy.Subscriber('plant_coordinates', PlantCoordinates, self.plant_callback)
        rospy.Timer(rospy.Duration(5.0), self.timer_callback)

        # Specify the path where you want to create the database file
        self.db_path = '/home/student/Desktop/Test/src/balise/src/robot_coordinates.db'

        # Initialize the database
        self.initialize_database()

    def detect_aruco(image_path):
    # Load the image
        img = cv2.imread(image_path)

    # ArUco dictionary and parameters (replace with the values from the previous group)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        aruco_parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_parameters)

    # Example: Extract x, y, theta from the detected ArUco markers
        if corners and ids:
        # Calculate x, y, theta based on the pose estimation
            x, y, theta = calculate_robot_coordinates(corners, ids)

            return x, y, theta, ids[0]  # Assuming a single ArUco marker, change accordingly
    
    def timer_callback(self, event):
        # Capture video from your computer's webcam
        cap = cv2.VideoCapture(0)  # Use 0 for the default webcam

        # Read a frame from the webcam
        ret, frame = cap.read()

        # Release the video capture object
        cap.release()

        # Check if the frame is successfully captured
        if not ret:
            rospy.logerr("Error capturing frame from the webcam.")
            return

        # Save the captured frame as an image
        image_path = "/home/student/Desktop/Test/src/balise/src/.jpg"
        cv2.imwrite(image_path, frame)

        # Detect ArUco markers and get robot coordinates
        x, y, theta, _id = detect_aruco(image_path)

        # Create a RobotCoordinates message
        robot_msg = RobotCoordinates()
        robot_msg.x = x
        robot_msg.y = y
        robot_msg.theta = theta
        robot_msg.id = _id

        rospy.loginfo(f"Publishing robot: ID {_id} Coordinates: {x}, {y}, {theta}")
        self.publisher.publish(robot_msg)

        # Update the database (if needed)
        self.update_database(x, y, theta, _id)

    def calculate_robot_coordinates(corners, ids):
        matrix_coeff = np.array([[2.5959453540558084e+03, 0., 1.7015437024875371e+03],
                            [0., 2.5981150581304091e+03, 1.2459640171617973e+03],
                            [0., 0., 1.]])

        distortion_coeff = np.array([2.0203034110644411e-01, -3.8937675520950621e-01,
                                 2.8240913779598222e-03, 5.4936365788275619e-03,
                                 -9.1462806356767012e-02])

    # Estimate the pose of the first detected marker
        rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.02,
                                                                    matrix_coeff,
                                                                    distortion_coeff)

    # Extract x, y, theta from the pose information
        x, y, z = tvec[0][0]
        theta = np.degrees(np.arctan2(rvec[0][0][1], rvec[0][0][0]))

        return x, y, theta

    def plant_callback(self, msg):
        rospy.loginfo(f"Received plant: ID {msg.id}, Coordinates: {msg.x}, {msg.y}, {msg.theta}")
        self.update_database_p(msg.x, msg.y, msg.theta, msg.id)

    def initialize_database(self):
        connection = sqlite3.connect(self.db_path)
        cursor = connection.cursor()

        # Create the 'robot_coordinates' table if it doesn't exist
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS robot_coordinates (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL,
                y REAL,
                theta REAL
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS plant_coordinates (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL,
                y REAL,
                theta REAL
            )
        ''')

        connection.commit()
        connection.close()

    def update_database(self, x, y, theta, _id):
        try:
            connection = sqlite3.connect(self.db_path)
            cursor = connection.cursor()

            cursor.execute('DELETE FROM robot_coordinates WHERE id = ?', (_id,))
            cursor.execute('INSERT INTO robot_coordinates (id, x, y, theta) VALUES (?, ?, ?, ?)', (_id, x, y, theta))

            connection.commit()

        except sqlite3.Error as e:
            rospy.logerr(f"Error updating database: {e}")

        finally:
            connection.close()

    def update_database_p(self, x, y, theta, _id):
        try:
            connection = sqlite3.connect(self.db_path)
            cursor = connection.cursor()

            cursor.execute('DELETE FROM plant_coordinates WHERE id = ?', (_id,))
            cursor.execute('INSERT INTO plant_coordinates (id, x, y, theta) VALUES (?, ?, ?, ?)', (_id, x, y, theta))

            connection.commit()

        except sqlite3.Error as e:
            rospy.logerr(f"Error updating database: {e}")

        finally:
            connection.close()

def main():
    robot_publisher = RobotPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
