import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2
import numpy as np
 
class FindBlue(Node):
    def __init__(self):
        super().__init__('find_blue')
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.get_img_cb, 10)
        self.cv_img = cv2.imread("/home/lej/blank.png", cv2.IMREAD_COLOR)
        self.bridge = CvBridge()
   
    def get_img_cb(self, msg):
        #self.get_logger().info('---')
        self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    node = FindBlue()
  
    try:    
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            frame = node.cv_img
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([100,100,120])
            upper_blue = np.array([150,255,255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            res = cv2.bitwise_and(frame, frame, mask=mask)
            
            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            _, bin = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            largest_contour = None
            largest_area = 0    
            COLOR = (0, 255, 0)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > largest_area:
                    largest_area = area
                    largest_contour = cnt                
                    # draw bounding box with green line
                if largest_contour is not None:
                    #area = cv2.contourArea(cnt)
                    if largest_area > 500:  # draw only larger than 500
                        x, y, width, height = cv2.boundingRect(largest_contour)       
                        cv2.rectangle(frame, (x, y), (x + width, y + height), COLOR, 2)
                        center_x = x + width//2
                        center_y = y + height//2
                        print("center: ( %s, %s )"%(center_x, center_y))
            cv2.imshow("VideoFrame",frame)       # show original frame
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
