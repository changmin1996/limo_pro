# basic
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# module for navigation
from nav2_msgs.action import NavigateThroughPoses
from limo_ros2_application.nav_utils import NavPose, NavigateStatus
from geometry_msgs.msg import PoseStamped

# for math
import numpy as np

class DriveThroughCircle(Node):
    def __init__(self):
        super().__init__('drive_through_circle')

        # set action client
        self.action_client_ = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses')

        # set waypoint
        self.goals_msg = NavigateThroughPoses.Goal()
        self.make_points_on_circle(3.3, 0.0, 0.5) # variable for center and radius

        # set timer, evry 1 sec run callback
        self.timer = self.create_timer(1, self.timer_callback)

        # to check navigation status first set default
        self.limo_state = NavigateStatus(0)
    
    def timer_callback(self):
        # send goal when the limo is not navigating
        if not self.limo_state == NavigateStatus.ACTIVE:
            self.send_goal()
    
    def send_goal(self):
        # send goal
        self.action_client_.wait_for_server()
        self._send_goal_future = self.action_client_.send_goal_async(self.goals_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        # get respownse if the request was accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        # if accepted then update limo state and goal index
        self.limo_state = NavigateStatus.ACTIVE
    
    def get_result_callback(self, future):
        # after reach goal and get result than update limo state
        self.limo_state = NavigateStatus.GOAL

    def make_points_on_circle(self, x_center=0.0, y_center=0.0, radius=1.0):
        tmp_point_ = NavPose()
        tmp_poseStamped = PoseStamped()
        tmp_poseStamped.header.frame_id = 'map'
        for i in range(8): # set point in evry 45 degree so it is 8
            # calculate angle in radians
            angle_ = 2 * np.pi * i / 8
            
            # calculate orientation, 
            # which is perpendicular to the angle towards the center
            # and normalize it to become between -pi ~ pi 
            orientation_ = angle_ + np.pi / 2
            orientation_ = self.normalize_angle(orientation_)

            # calculate coordinate
            x_ = x_center + radius * np.cos(angle_)
            y_ = y_center + radius * np.sin(angle_)

            # set the temp point
            tmp_point_.set_pose(x_, y_, orientation_)
            tmp_poseStamped.pose = tmp_point_.get_pose()

            self.goals_msg.poses.append(tmp_poseStamped)
    
    # set angle to -pi ~ pi
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    drive_through_circle = DriveThroughCircle()

    rclpy.spin(drive_through_circle)

    drive_through_circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()