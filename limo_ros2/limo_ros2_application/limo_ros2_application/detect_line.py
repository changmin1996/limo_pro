import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectLine(Node):
    def __init__(self):
        super().__init__('detect_line')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.subscription
        self.br = CvBridge()
        self.roi_publisher = self.create_publisher(Image, 'roi_image', 10)
        self.mask_publisher = self.create_publisher(Image, 'mask_image', 10)
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.dis_publisher = self.create_publisher(Int32, 'distance_y', 10)

        self.yellow_lane_low = np.array([0, 90, 100])
        self.yellow_lane_high = np.array([60, 220, 255])
        

    def image_callback(self, msg):
        image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
        
        roi_ = image_[350:400, 0:320]
        hls = cv2.cvtColor(roi_, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.yellow_lane_low, self.yellow_lane_high)
        
        M = cv2.moments(mask_yellow)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cy = 350 + cy
            image_ = cv2.circle(image_, (cx, cy), 10,(255, 0, 0), -1)
            distance_to_ref = 250 -cx
        else:
            distance_to_ref = -999

        dis = Int32()
        dis.data = distance_to_ref
        
        self.dis_publisher.publish(dis)
        self.roi_publisher.publish(self.br.cv2_to_imgmsg(roi_,'bgr8'))
        self.debug_publisher.publish(self.br.cv2_to_imgmsg(image_,'bgr8'))
        self.mask_publisher.publish(self.br.cv2_to_imgmsg(mask_yellow,'mono8'))

def main(args=None):
    rclpy.init(args=args)

    detect_line = DetectLine()

    rclpy.spin(detect_line)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detect_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()