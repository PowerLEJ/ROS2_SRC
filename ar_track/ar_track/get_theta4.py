import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion #, quaternion_from_euler
from ar_track.move_tb3 import MoveTB3
from ar_track.delay import Delay

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

R = 1.5708


class GetTheta(Node):
    """   
                                                    ////////////| ar_marker |////////////
            y                      z                --------+---------+---------+--------
            ^  x                   ^                        |     R-0/|\R-0    R|
            | /                    |                        |       /0|0\       |
     marker |/                     | robot                  |      /  |  \      |
            +------> z    x <------+                        |     /   |   \     |
                                  /                         |  dist   |  dist   |
                                 /                          |   /     |     \   |
                                y                           |  /      |      \  |
                                                            | /       |       \0|
                                                            |/R-0    R|R    R-0\|
    pose.x = position.z                             (0 < O) x---------+---------x (0 > O)
    pose.y = position.x              [0]roll                ^                   ^ 
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.get_marker_pose_, qos_profile)
        
        self.pose = Pose()        
        self.theta   = 0.0
        
        
    def get_marker_pose_(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        if len(msg.marker_ids) == 0:    # no marker found
            self.target_found = False
        
        else: # if len(msg.marker_ids) != 0: # marker found at least 1EA
            self.target_found = True
            for i in range(len(msg.marker_ids)):
            
                if msg.marker_ids[i] == TARGET_ID:  # target marker found
                    self.pose  = msg.poses[i]
                    self.theta = self.get_theta(self.pose)
                else:
                    pass            
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta
        
def main(args=None):

    rclpy.init(args=args)
    node = GetTheta()
    delay = Delay()
    try:   
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            print("theta = %s(deg)" %round(degrees(node.theta),1))
            delay.ms(500)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

