import rclpy
from rclpy.node import Node
from my_robot_controller.msg import RobotCoordinates, PlantCoordinates
import sqlite3
import random


class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')
        self.publisher_ = self.create_publisher(RobotCoordinates, 'robot_coordinates', 10)
        self.plant_subscription = self.create_subscription(
            PlantCoordinates,
            'plant_coordinates',
            self.plant_callback,
            10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Specify the path where you want to create the database file
        self.db_path = '/home/student/ros2_workspace/src/my_robot_controller/src/robot_coordinates.db'

        # Initialize the database
        self.initialize_database()

    def timer_callback(self):
        x = random.uniform(0, 100)
        y = random.uniform(0, 100)
        theta = random.uniform(0, 360)
        id = 1

        robot_msg = RobotCoordinates()
        robot_msg.x = x
        robot_msg.y = y
        robot_msg.theta = theta
        robot_msg.id = id
        self.get_logger().info(f"Publishing robot: ID {id} Coordinates: {x}, {y}, {theta}")
        self.publisher_.publish(robot_msg)
        self.update_database(x, y, theta, id)

    def plant_callback(self, msg):
        self.get_logger().info(f"Received plant: ID {msg.id}, Coordinates: {msg.x}, {msg.y}, {msg.theta}")

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

    def update_database(self, x, y, theta, id):
        try:
            connection = sqlite3.connect(self.db_path)
            cursor = connection.cursor()

            # Update the 'robot_coordinates' table
            cursor.execute('DELETE FROM robot_coordinates WHERE id = ?', (id,))
            cursor.execute('INSERT INTO robot_coordinates (id, x, y, theta) VALUES (?, ?, ?, ?)', (id, x, y, theta))

            # Assuming you have 'plant_msg' available with the correct data
            plant_msg = self.get_plant_msg(id)  # Replace this with your actual method
            
            if plant_msg:
                # Update the 'plant_coordinates' table
                cursor.execute('DELETE FROM plant_coordinates WHERE id = ?', (id,))
                cursor.execute('INSERT INTO plant_coordinates (id, x, y, theta) VALUES (?, ?, ?, ?)',
                               (plant_msg.id, plant_msg.x, plant_msg.y, plant_msg.theta))

            connection.commit()

        except sqlite3.Error as e:
            self.get_logger().error(f"Error updating database: {e}")

        finally:
            connection.close()

    def get_plant_msg(self, id):
        # This method should retrieve the corresponding PlantCoordinates message
        plant_msg = PlantCoordinates()

        return plant_msg


def main(args=None):
    rclpy.init(args=args)

    robot_publisher = RobotPublisher()

    rclpy.spin(robot_publisher)

    # Destroy the node explicitly
    robot_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
