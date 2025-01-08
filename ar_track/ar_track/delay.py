import rclpy
from rclpy.node import Node

class Delay(Node):
       
    def __init__(self):
        super().__init__('delay_ms')
        periode = 0.001
        self.create_timer(periode, self.timer_cb)
        
        self.timer_cnt = 0
    
    def timer_cb(self):
        self.timer_cnt = self.timer_cnt + 1
    
    def ms(self,msec):
        duration = self.timer_cnt + msec
        
        while self.timer_cnt < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            pass

            

