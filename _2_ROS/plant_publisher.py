import rclpy
from rclpy.node import Node
from my_robot_controller.msg import PlantCoordinates
import random


class PlantPublisher(Node):
    def __init__(self):
        super().__init__('plant_publisher')
        self.publisher_ = self.create_publisher(PlantCoordinates, 'plant_coordinates', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        plant_msg = PlantCoordinates()
        plant_msg.id = random.randint(1, 36)  # Replace with your actual ID range
        plant_msg.x = random.uniform(0, 100)
        plant_msg.y = random.uniform(0, 100)
        plant_msg.theta = random.uniform(0, 360)

        self.get_logger().info(f"Publishing plant: ID {plant_msg.id}, Coordinates: {plant_msg.x}, {plant_msg.y}, {plant_msg.theta}")
        self.publisher_.publish(plant_msg)


def main(args=None):
    rclpy.init(args=args)

    plant_publisher = PlantPublisher()

    rclpy.spin(plant_publisher)

    # Destroy the node explicitly
    plant_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
