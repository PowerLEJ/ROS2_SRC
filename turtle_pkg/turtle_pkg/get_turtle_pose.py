import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class SubTurtlePose(Node):

    def __init__(self):
        super().__init__("get_turtle_pose")
        self.create_subscription(
            Pose, "/turtle1/pose", self.get_pose, 10
        )
        self.turtle_pose = Pose()

    def get_pose(self, msg):
        self.turtle_pose = msg
        # print("x = %s, y = %s, th = %s" % (msg.x, msg.y, msg.theta))


def main(args=None):
    rclpy.init(args=args)
    
    node = SubTurtlePose()
    # rclpy.spin(node)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            print("x = %s, y = %s, th = %s" % (node.turtle_pose.x, node.turtle_pose.y, node.turtle_pose.theta))

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()