import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ArUcoMarkerFollower(Node):
    def __init__(self):
        super().__init__('aruco_marker_follower')

        # Publisher to control robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Timer for spinning
        self.timer = self.create_timer(0.1, self.spin)

        self.bridge = CvBridge()
        self.marker_detected = False
        self.original_pose = None
        self.start_time = None
        self.returning_to_origin = False

        self.moving_to_marker = False

    def spin(self):
        if self.marker_detected:
            return

        twist = Twist()
        twist.angular.z = 0.5  # Rotate counterclockwise
        self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        if self.marker_detected:
            return

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Detect ArUco markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            self.marker_detected = True
            self.cmd_vel_pub.publish(Twist())  # Stop spinning

            # Process the first marker detected
            marker_center = np.mean(corners[0][0], axis=0)  # Average of corner points
            image_center = (cv_image.shape[1] / 2, cv_image.shape[0] / 2)

            # Calculate offset from image center
            offset_x = marker_center[0] - image_center[0]
            offset_y = marker_center[1] - image_center[1]

            self.move_to_marker(offset_x, offset_y)

    def move_to_marker(self, offset_x, offset_y):
        twist = Twist()

        # Rotate towards the marker
        twist.angular.z = -0.002 * offset_x

        # Move forward if aligned
        if abs(offset_x) < 50:
            twist.linear.x = 0.2

        self.cmd_vel_pub.publish(twist)

        # Stop after reaching the marker
        if abs(offset_x) < 50 and abs(offset_y) < 50:
            self.cmd_vel_pub.publish(Twist())
            self.start_timer_to_return()

    def start_timer_to_return(self):
        self.start_time = time.time()
        self.returning_to_origin = True

    def return_to_origin(self):
        if not self.returning_to_origin:
            return

        if time.time() - self.start_time < 10:
            return

        twist = Twist()
        twist.linear.x = -0.2  # Move backwards
        self.cmd_vel_pub.publish(twist)

        # Stop after a certain duration (placeholder for actual localization logic)
        if time.time() - self.start_time > 15:
            self.cmd_vel_pub.publish(Twist())
            self.returning_to_origin = False


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoMarkerFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
