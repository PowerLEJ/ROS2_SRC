import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubNGo(Node):

    def __init__(self):
        super().__init__('sub_n_go')
        self.create_subscription(String, '/nav_msg', self.get_nav_msg, 10)
        self.str_msg = String()

    def get_nav_msg(self, msg):
        self.str_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = SubNGo()

    try:
       while rclpy.ok():
           rclpy.spin_once(node, timeout_sec=0.1)
           
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()