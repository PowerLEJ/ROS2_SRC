import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile


class Delay(Node):
      
    def __init__(self):
        
        qos_profile = QoSProfile(depth=10)
        periode = 1.0
        self.create_timer(periode, self.timer_cb)
        self.tim_cnt = 0
    
    def timer_cb(self):
        self.tim_cnt = self.tim_cnt + 1
    
    def ms(self, tim_ms):
        duration = self.tim_cnt + tim_ms
        
        while self.tim_cnt < duration:
            pass
        
        
        
def main(args=None):

    rclpy.init(args=args)
    node = TimerEx()
    rclpy.spin(node)
    
    try:    
        while rclpy.ok():
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
            node.tb3.rotate( -angle * .9)
        elif node.th_ref < radians(-10):
            node.tb3.rotate(angle * .97)
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
        node.tb3.straight(dist2)
        print("\n----- 6_arrived lifting position!\n") ####################
        
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

