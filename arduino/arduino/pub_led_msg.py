import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .getchar import Getchar


class PubLedMsg(Node):

    def __init__(self):
        super().__init__('pub_led_msg')
        
def main(args=None):
    rclpy.init(args=args)

    node = PubLedMsg()
    msg = String()
    kb = Getchar()

    pub = node.create_publisher(String, '/led_msg', 10)
    
    try:
        while rclpy.ok():
            key = kb.getch()

            if key == '1':
                print("on")
                msg.data = "on"
            elif key == '0':
                print("off")
                msg.data = "off"
            else:
                pass

            pub.publish(msg)

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()