import rclpy, serial
from rclpy.node import Node
from std_msgs.msg import String

sp = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

class ControlLed(Node):

   def __init__(self):
        super().__init__('ctrl_led')      
        self.create_subscription(String, '/led_msg', self.get_led_msg, 10)
        self.tx_dat = '' # 전송할 데이터

   def get_led_msg(self, msg):
        if msg.data == "on" :
            self.tx_dat = '1'
        elif msg.data == "off":
            self.tx_dat = "0"
        else:
            pass            
        
        # sp.write(str.encode(self.tx_dat))
        sp.write(self.tx_dat.encode())


def main(args=None):

    rclpy.init(args=args)
    node = ControlLed()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()