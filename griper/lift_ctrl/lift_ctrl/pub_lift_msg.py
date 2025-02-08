import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .getchar import Getchar


class LiftControl(Node):

    def __init__(self):
        super().__init__('pub_lift_ctrl')


def main(args=None):

    rclpy.init(args=args)
    
    try:

        node = LiftControl()
        pub = node.create_publisher(String, 'lift_msg', 10)
        
        kb = Getchar()
        msg = String()
        key = ''
        
        while rclpy.ok():
        
            key = kb.getch()
            
            if key == '1':
                msg.data = 'lift_up'
            
            elif key == '0':
                msg.data = 'lift_down'
            else:
                pass
            
            pub.publish(msg)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
