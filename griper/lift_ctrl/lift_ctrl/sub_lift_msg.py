import rclpy, serial
from rclpy.node import Node

from std_msgs.msg import String

sp  = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)


class SubLiftMsg(Node):

    def __init__(self):
        super().__init__('sub_lift_msg')
        
        self.create_subscription(String, 'lift_msg', self.get_lift_msg_cb, 10)

    def get_lift_msg_cb(self, msg):
        if msg.data == "lift_up":
            print("subscribe msg is 'lift_up'")
            sp.write(b'1') 
        elif msg.data == "lift_down":
            print("subscribe msg is 'lift_down'")
            sp.write(b'0')
        else:
            pass
        


def main(args=None):

    rclpy.init(args=args)
    
    try:

        node = SubLiftMsg()
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
