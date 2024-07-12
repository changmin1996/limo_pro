import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from limo_ros2_application.nav_utils import euler_from_quaternion


class RotateAbsolute(Node):
    def __init__(self):
        super().__init__('rotate_absolute')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

            
    def imu_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion(msg.orientation.x,
                                                   msg.orientation.y,
                                                   msg.orientation.z, 
                                                   msg.orientation.w)
        self.get_logger().info('roll: %f, pitch: %f, yaw: %f' % (roll, pitch, yaw))        

def main(args=None):
    rclpy.init(args=args)

    rotate_absolute = RotateAbsolute()

    rclpy.spin(rotate_absolute)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_absolute.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()