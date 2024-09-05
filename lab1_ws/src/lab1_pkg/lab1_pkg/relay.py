import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

class DrivePublisher(Node):
    def __init__(self):
        super().__init__('DrivePublisher')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

        msg.drive.speed *= float(3)
        msg.drive.steering_angle *= float(3)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DrivePublisher()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()