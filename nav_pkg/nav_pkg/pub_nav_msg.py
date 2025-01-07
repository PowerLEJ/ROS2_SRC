import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .getchar import Getchar


class PubNavMsg(Node):          
    def __init__(self):
        super().__init__('pub_nav_msg')
        

def main(args=None):
    rclpy.init(args=args)

    node = PubNavMsg()
    str_msg = String()
    kb = Getchar()

    pub = node.create_publisher(String, '/nav_msg', 10)

    key = ' '

    try:
        while rclpy.ok():
            key = kb.getch()

            if key == '1':
                str_msg.data = "point1"
            elif key == '2':
                str_msg.data = "point2"
            elif key == '3':
                str_msg.data = "point3"
            elif key == '4':
                str_msg.data = "point4"
            else:
                str_msg.data = ""
                pass

            pub.publish(str_msg)
            print(str_msg.data)

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()