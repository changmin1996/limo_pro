from geometry_msgs.msg import Pose
from enum import Enum
import numpy as np
import math

def quaternion_from_euler(a_1, a_2, a_3):
    # get roll, pitch, yaw and divide by 2
    a_1 /= 2.0 
    a_2 /= 2.0
    a_3 /= 2.0
    
    # calculate cos and sin
    c_1 = math.cos(a_1)
    s_1 = math.sin(a_1)
    c_2 = math.cos(a_2)
    s_2 = math.sin(a_2)
    c_3 = math.cos(a_3)
    s_3 = math.sin(a_3)

    # calculate quaternion with variable
    # q = q[0]i + q[1]j + q[2]k + q[3]
    q = np.empty((4, ))
    q[0] = s_1 * c_2 * c_3 - c_1 * s_2 * s_3
    q[1] = c_1 * s_2 * c_3 + s_1 * c_2 * s_3
    q[2] = c_1 * c_2 * s_3 - s_1 * s_2 * c_3
    q[3] = c_1 * c_2 * c_3 + s_1 * s_2 * s_3

    return q

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

# you can check the limo status with enum class
class NavigateStatus(Enum):
    DEFAULT = 0
    PENDING = 1
    ACTIVE = 2
    GOAL = 3
    FAILED = 4