from geometry_msgs.msg import Pose
from limo_ros2_application.quaternion_from_euler import quaternion_from_euler

class NavPose:
    def __init__(self):
        self.pos_ = Pose()
        
    def set_pose(self, x, y, theta):
        self.pos_.position.x = x
        self.pos_.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        self.pos_.orientation.x = q[0]
        self.pos_.orientation.y = q[1]
        self.pos_.orientation.z = q[2]
        self.pos_.orientation.w = q[3]

    def get_pose(self):
        return self.pos_