import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool


class LimoControl(Node):

    def __init__(self):
        super().__init__('limo_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.e_stop_subscription = self.create_subscription(
            Bool,
            'e_stop',
            self.e_stop_callback,
            10)
        self.distance_subscription = self.create_subscription(
            Int32,
            'distance_y',
            self.distance_callback,
            10)
        self.e_stop_subscription
        self.distance_subscription
        self.e_stop_flag = False
        self.gap = 0
    
    def e_stop_callback(self, msg):
        self.e_stop_flag = msg.data
    
    def distance_callback(self, msg):
        self.gap = msg.data

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = self.gap * 0.007

        if self.gap < -900:
            msg.linear.x = 0.1
            msg.angular.z - 0.0
        elif self.e_stop_flag :
            msg.linear.x = 0.0
            msg.angular.z - 0.0
        else:
            pass
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    limo_control = LimoControl()

    rclpy.spin(limo_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    limo_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()