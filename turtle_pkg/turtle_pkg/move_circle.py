import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveCircle(Node):

    def __init__(self):
        super().__init__('move_circle')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
def main(args=None):
    rclpy.init(args=args)

    node = MoveCircle()

    tw = Twist()
    tw.linear.x = 2.0
    tw.angular.z = 2.0

    try:
        while rclpy.ok():
            node.pub.publish(tw)
            
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
