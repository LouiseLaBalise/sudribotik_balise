import rclpy
from rclpy.node import Node
from my_robot_controller.msg import RobotCoordinates
from my_robot_controller.msg import PlantCoordinates


class RobotSubscriber(Node):
    def __init__(self):
        super().__init__('robot_subscriber')
        self.subscription = self.create_subscription(
            RobotCoordinates,
            'robot_coordinates',
            self.robot_callback,
            10)
        self.plant_subscription = self.create_subscription(
            PlantCoordinates,
            'plant_coordinates',
            self.plant_callback,
            10)

    def robot_callback(self, msg):
        self.get_logger().info(f"Received robot: ID {msg.id}, Coordinates: {msg.x}, {msg.y}, {msg.theta}")

    def plant_callback(self, msg):
        self.get_logger().info(f"Received plant: ID {msg.id}, Coordinates: {msg.x}, {msg.y}, {msg.theta}")


def main(args=None):
    rclpy.init(args=args)

    robot_subscriber = RobotSubscriber()

    rclpy.spin(robot_subscriber)

    # Destroy the node explicitly
    robot_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
