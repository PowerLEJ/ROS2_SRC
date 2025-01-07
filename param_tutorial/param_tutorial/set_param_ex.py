import rclpy, os
from rclpy.node import Node
from std_msgs.msg import String
from .getchar import Getchar


class SetParamEx(Node):

    def __init__(self):
        super().__init__('set_param_ex')
        
def main(args=None):
    rclpy.init(args=args)

    node = SetParamEx()
    kb = Getchar()

    try:
        while rclpy.ok():
            key = kb.getch()

            if key == '1':
                print("go")
                os.system("ros2 param set /move_by_param go_turtle go")
            elif key == '0':
                print("stop")
                os.system("ros2 param set /move_by_param go_turtle stop")
            else:
                pass

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()