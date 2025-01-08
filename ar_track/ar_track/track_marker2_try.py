import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion #, quaternion_from_euler
from ar_track.move_tb3 import MoveTB3

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

R = 1.5708


class TrackMarker(Node):
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
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose  = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
            
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_lift = self.create_publisher(String, '/lift__msg', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)
        
        self.pose = Pose()
        self.tw   = Twist()
        self.tb3  = MoveTB3()
        self.lift_msg = String()
        
        self.theta   = 0.0
        self.dir     = 0
        self.th_ref  = 0.0
        self.z_ref   = 0.0
        self.cnt_sec = 0
        
        self.target_found = False
        
        
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
        
            for i in range(len(msg.marker_ids)):
            
                if msg.marker_ids[i] == TARGET_ID:  # target marker found
                    if self.target_found == False:
                        self.target_found = True                        
                    self.pose  = msg.poses[i]
                    self.theta = self.get_theta(self.pose)
                else:
                    self.target_found = False            
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta
    
    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1    
        
    def pub_lift_msg(self, lift_msg):
        msg = String()
        msg.data = lift_msg
        self.pub_lift.publish(msg)
    
    def stop_move(self):
        self.tw.linear.z = self.tw.angular.z = 0.0
        self.pub_tw.publish(self.tw)      

    # def adjust_distance(self):
    #     dist_ref = 0.15  # Desired distance (15 cm)
    #     margin = 0.025   # Acceptable margin (2.5 cm)

    #     while rclpy.ok():
    #         rclpy.spin_once(self, timeout_sec=0.1)

    #         if self.pose.position.z > dist_ref + margin:
    #             self.tw.linear.x = 0.05
    #             print("forward")
    #         elif self.pose.position.z < dist_ref - margin:
    #             self.tw.linear.x = -0.05
    #             print("back")
    #         else:
    #             self.tw.linear.x = 0.0
    #             print("stop")
    #             self.pub_tw.publish(self.tw)
    #             print("stop")
    #             break
            
    #         self.pub_tw.publish(self.tw)

    def adjust_distance(self):
        dist_ref = 0.15  # Desired distance (15 cm)
        margin = 0.025   # Acceptable margin (2.5 cm)
        marker_lost = False  # Flag to track marker visibility

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self.marker_visible():
                marker_lost = True
                print("Marker lost. Searching for marker...")
                self.search_marker()  # Function to search for the marker

            if marker_lost:
                # If marker is lost, do not move forward until it's found
                continue
            
            # Adjust movement based on distance
            if self.pose.position.z > dist_ref + margin:
                self.tw.linear.x = 0.05
                print("forward")
            elif self.pose.position.z < dist_ref - margin:
                self.tw.linear.x = -0.05
                print("back")
            else:
                self.tw.linear.x = 0.0
                print("stop")
                self.pub_tw.publish(self.tw)
                print("stop")
                break
            
            self.pub_tw.publish(self.tw)

    def marker_visible(self):
        # Check if marker is detected (you would need to implement this)
        # Return True if visible, False otherwise
        return True  # Placeholder

    def search_marker(self):
        # Code to search for the marker, e.g., rotate or move to known position
        print("Searching for marker...")
        self.tw.angular.z = 0.2  # Rotate slowly
        self.pub_tw.publish(self.tw)
        rclpy.sleep(1)  # Give some time for rotation/searching
        self.tw.angular.z = 0.0  # Stop rotating
        self.pub_tw.publish(self.tw)
        print("Marker search complete.")
        
        
def main(args=None):

    rclpy.init(args=args)
    node = TrackMarker()
    
    node.tw.angular.z = 0.5 * ANG_SPEED
    
    try:    
        while rclpy.ok():
            if node.theta != 0.0:   break   # this means target marker found
            node.pub_tw.publish(node.tw)
            rclpy.spin_once(node, timeout_sec=0.1)
        
        node.stop_move()
        print("\n----- 1_target marker found!\n") ###########################
        
        while node.pose.position.x < -0.0175 or node.pose.position.x >  0.0175:
            rclpy.spin_once(node, timeout_sec=0.1)
            if   node.pose.position.x < -0.0155:
                node.tw.angular.z =  0.175 * ANG_SPEED
            else:# node.pose.position.x >  0.025:
                node.tw.angular.z = -0.125 * ANG_SPEED
            node.pub_tw.publish(node.tw)
            rclpy.spin_once(node, timeout_sec=0.1)
        
        node.stop_move()        
        print("\n----- 2_arrived reference position!\n") ####################
        
        node.th_ref = node.theta
        node.z_ref  = node.pose.position.z
        if node.th_ref >= 0:
            node.dir =  1
        else:
            node.dir = -1
        
        angle = R - node.th_ref
                                
        if angle > R:
            angle = pi - angle
        
        if   node.th_ref > radians( 10):
            node.tb3.rotate( angle * .9)
        elif node.th_ref < radians(-10):
            node.tb3.rotate(-angle * .97)
        else:
            pass        
        print("\n----- 3_1st rotation finished!\n") #########################
        
        dist1 = abs(node.z_ref * sin(node.th_ref) * 1.125)
        node.tb3.straight(dist1)
        print("\n----- 4_move to front of marker end!\n") ###################
        
        if   node.th_ref >  radians(10):
            node.tb3.rotate(-R * 0.875)
        elif node.th_ref < -radians(10):
            node.tb3.rotate( R)
        else:
            pass        
        print("\n----- 5_2nd rotation finished!\n") #########################
        
        while node.pose.position.x < -0.0025 or node.pose.position.x >  0.0025:
            if   node.pose.position.x < -0.0025:
                node.tw.angular.z =  0.075 * ANG_SPEED
            elif node.pose.position.x >  0.0025:
                node.tw.angular.z = -0.075 * ANG_SPEED
            else:
                node.tw.angular.z =  0.0
                
            node.pub_tw.publish(node.tw)                
            rclpy.spin_once(node, timeout_sec=0.02)
            
        dist2 = node.pose.position.z - 0.185
        # node.tb3.straight(dist2)
        print("\n----- 6_arrived lifting position!\n") ####################
        
        print("\n----- 6.1_adjusting distance...\n")
        node.adjust_distance()
        print("\n----- 6.2_adjustment finished!\n")

        node.pub_lift_msg("lift_up")
        duration = node.cnt_sec + 10
        
        while node.cnt_sec < duration: 
            print(duration - node.cnt_sec)               
            rclpy.spin_once(node, timeout_sec=1.0)
        print("\n----- 7_finished loading!\n") ############################     
        
        node.tb3.straight(-dist2)
        node.tb3.rotate(R * node.dir)
        node.tb3.straight(-dist1)
        print("\n----- 8_arrived starting point!\n") ######################
        
        node.pub_lift_msg("lift_down")
        duration = node.cnt_sec + 8
        
        while node.cnt_sec < duration: 
            print(duration - node.cnt_sec)               
            rclpy.spin_once(node, timeout_sec=1.0)
        print("\n----- 7_finished unloading!\n") ###########################       
        
        node.tb3.straight(-0.1)
        node.tb3.rotate(R * node.dir * -1)
        
        sys.exit(1)
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

