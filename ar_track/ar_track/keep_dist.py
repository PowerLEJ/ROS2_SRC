import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion #, quaternion_from_euler
#from ar_track.move_tb3 import MoveTB3
from ar_track.delay import Delay

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

R = 1.5708


class KeepDist(Node):
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
    pose.y = position.x              [0]roll    (pos.x > O) ^                   ^ (pos.x < O)
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('keep_dist')
        qos_profile = QoSProfile(depth=10)
        
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.get_dist_, qos_profile)            
        
        self.pose = Pose()
        self.dist = 0.0        
        
    def get_dist_(self, msg):
        if len(msg.marker_ids) == 0:    # no marker found
            pass
        
        else: # if len(msg.marker_ids) != 0: # marker found at least 1EA
        
            for i in range(len(msg.marker_ids)):
            
                if msg.marker_ids[i] == TARGET_ID:  # target marker found
                    self.pose = msg.poses[i]
                    self.dist = msg.poses[i].position.z
                else:
                    pass
        
def main(args=None):

    rclpy.init(args=args)
    node = KeepDist()
    dist_ref = 0.15  #(15.0cm)
    margin   = 0.025 #( 2.5cm)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tw = Twist()
    
    
    try:    
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.dist > dist_ref + margin:
                tw.linear.x = 0.05
                print("foward")
            elif node.dist < dist_ref- margin:
                tw.linear.x = -0.05
                print("back")
            else:
                tw.linear.x = 0.0
                print("stop")
            pub.publish(tw)
            
            #print("distance to marker = %s(m)" %node.dist)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()
