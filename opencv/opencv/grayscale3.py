import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2
import numpy as np
 
class ImageConvertor(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.get_img_cb, 10)
        self.cv_img = cv2.imread("/home/leg/blank.png", cv2.IMREAD_COLOR)
        self.bridge = CvBridge()
   
    def get_img_cb(self, msg):
        #self.get_logger().info('---')
        self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    node = ImageConvertor()
  
    try:    
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            img_gray = cv2.cvtColor(node.cv_img, cv2.COLOR_BGR2GRAY)
            cv2.imshow("grayscale3", img_gray)
            k = cv2.waitKey(5) & 0xFF
        
            if k == 27:
                break
        node.destroy_node()
        rclpy.shutdown()
            
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        
        # Shutdown the ROS client library for Python
        rclpy.shutdown()
    
if __name__ == '__main__':
  main()
