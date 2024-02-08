import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectLine(Node):
    def __init__(self):
        # CV bridge
        self.br = CvBridge()

        # Subscribe camera data
        super().__init__('detect_line')
        self.subscription = self.create_subscription(
                            Image,
                            '/camera/color/image_raw', 
                            self.image_callback, 
                            rclpy.qos.qos_profile_sensor_data)
        self.subscription # to prevent from warning

        # Publish result (offset between reference distance and real distance)
        self.dis_publisher = self.create_publisher(Int32, 'distance_y', 10)

        # Publish Image for debugging
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Parameters (For Masking Lane, For the reference distance of lane, ROI)
        self.yellow_lane_low = np.array([0, 90, 100])
        self.yellow_lane_high = np.array([60, 220, 255])

        # Parameter For debugging sequence
        # 0: ROI
        # 1: Masking
        # 2: Moment of lane
        self.debug_sequence = 2
    
    def timer_callback(self):
        if self.debug_sequence == 0:
            self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.roi_,'bgr8'))
        elif self.debug_sequence == 1:
            self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.mask_yellow,'mono8'))
        else:    
            self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.image_,'bgr8'))
          
    def image_callback(self, msg):
        # convert opencv Mat type to image msg type
        self.image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
        
        # Make Region of Interest
        self.roi_ = self.image_[350:400, 0:320]

        # Masking the lane
        hls = cv2.cvtColor(roi_, cv2.COLOR_BGR2HLS)
        self.mask_yellow = cv2.inRange(hls, self.yellow_lane_low, self.yellow_lane_high)
        
        # Calculating the Moment of the lane
        M = cv2.moments(self.mask_yellow)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cy = 350 + cy
            self.image_ = cv2.circle(self.image_, (cx, cy), 10,(255, 0, 0), -1)
            distance_to_ref = 250 -cx
        else: # When limo cannot find lane publish garbage data
            distance_to_ref = -999

        # Publishing the offset between current distance with lane and reference distance
        dis = Int32()
        dis.data = distance_to_ref
        self.dis_publisher.publish(dis)

def main(args=None):
    rclpy.init(args=args)

    detect_line = DetectLine()
    rclpy.spin(detect_line)

    detect_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()