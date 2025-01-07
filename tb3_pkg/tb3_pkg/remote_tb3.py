import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .getchar import Getchar


MAX_LIN_SPD = 0.22 # 최대 이동 속도
MIN_LIN_SPD = -0.22 # 최소 이동 속도
LIN_SPD_STP = 0.01 # 이동 속도 스텝
MAX_ANG_SPD = 2.84 # 최대 회전 속도
ANG_SPD_STP = 0.1 # 회전 속도 스텝
MIN_ANG_SPD = -2.84 # 최소 회전 속도


msg = '''
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C or Q to quit
'''

class Remote_TB3(Node):

    def __init__(self):
        super().__init__('remote_tb3')
        
def main(args=None):
    rclpy.init(args=args)

    node = Remote_TB3()
    kb = Getchar()
    tw = Twist()

    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    try:
        print(msg)
        while rclpy.ok():

            key = kb.getch()

            if key == 'w':
                if tw.linear.x + LIN_SPD_STP <= MAX_LIN_SPD:
                    tw.linear.x = tw.linear.x + LIN_SPD_STP
                else:
                    tw.linear.x = MAX_LIN_SPD
            elif key == 'x':
                if tw.linear.x - LIN_SPD_STP >= MIN_LIN_SPD:
                    tw.linear.x = tw.linear.x - LIN_SPD_STP
                else:
                    tw.linear.x = MIN_LIN_SPD
            elif key == 'a':
                if tw.angular.z + ANG_SPD_STP <= MAX_ANG_SPD:
                    tw.angular.z = tw.angular.z + ANG_SPD_STP
                else:
                    tw.angular.z = MAX_ANG_SPD
            elif key == 'd':
                if tw.angular.z - ANG_SPD_STP >= MIN_ANG_SPD:
                    tw.angular.z = tw.angular.z - ANG_SPD_STP
                else:
                    tw.angular.z = MIN_ANG_SPD
            elif key == 's' or key == ' ':
                tw.linear.x = tw.angular.z = 0.0
            elif key == 'Q':
                print("Bye~")
                break
            else:
                pass

            pub.publish(tw)

            print("currently:	linear velocity %s	 angular velocity %s" % (tw.linear.x, tw.angular.z))
            
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
