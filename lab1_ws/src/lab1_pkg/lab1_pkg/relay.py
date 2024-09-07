import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

class DrivePublisher(Node):
    def __init__(self):
        super().__init__('DriveListener')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            'talker/drive',
            10)

        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self.get_logger().info('\nI heard V: "%s" and D: "%s".\n' \
                               % (msg.drive.speed, msg.drive.steering_angle))

        msg.drive.speed *= 3
        msg.drive.steering_angle *= 3

        self.publisher_.publish(msg)

        self.get_logger().info('\nUPDATED INFO V: "%s" and D: "%s" and published.\n' \
                               % (msg.drive.speed, msg.drive.steering_angle))

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