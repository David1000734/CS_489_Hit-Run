import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

class DrivePublisher(Node):
    def __init__(self):
        super().__init__('DrivePublisher')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.5      # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Declare parameters, Speed and Steering Angle
        self.declare_parameter('v', rclpy.Parameter.Type.DOUBLE)   # Speed
        self.declare_parameter('d', rclpy.Parameter.Type.DOUBLE)   # Steering Angle

    def timer_callback(self):
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = float(v)
        ack_msg.drive.steering_angle = float(d)
        self.publisher_.publish(ack_msg)

        self.get_logger().info('\nPublishing: "%s"\n\n' % ack_msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = DrivePublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
