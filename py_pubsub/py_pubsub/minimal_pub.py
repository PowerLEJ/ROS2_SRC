# ROS 2 라이브러리 임포트
import rclpy                           # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node            # ROS 2 노드 클래스
from std_msgs.msg import String        # ROS 2 표준 메시지 타입(String)

# 퍼블리셔 노드를 정의하는 클래스
class MinimalPublisher(Node):          
    def __init__(self):
        # 부모 클래스(Node)를 초기화하면서 노드 이름을 설정
        super().__init__('minimal_publisher')
        
        # 퍼블리셔 생성
        # - 메시지 타입: String
        # - 토픽 이름: 'hello'
        # - QoS 설정: 큐 크기 10
        self.publisher_ = self.create_publisher(String, 'hello', 10)
        
        # 타이머 설정
        # - 타이머 주기: 0.5초
        # - 타이머 콜백 함수: self.timer_callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 메시지 카운터 초기화
        self.i = 0

    # 타이머 콜백 함수
    # - 0.5초마다 호출되며 메시지를 생성하고 퍼블리셔를 통해 게시
    def timer_callback(self):
        # 새로운 String 메시지 생성
        msg = String()
        
        # 메시지 데이터 설정 (예: 'Hello World: 0')
        msg.data = 'Hello World: %d' % self.i
        
        # 메시지 게시
        self.publisher_.publish(msg)
        
        # 게시된 메시지를 로그로 출력
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        # 메시지 카운터 증가
        self.i += 1

# 메인 함수
def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    
    # 퍼블리셔 노드 인스턴스 생성
    minimal_publisher = MinimalPublisher()
    
    # 노드를 실행하여 퍼블리싱 시작
    rclpy.spin(minimal_publisher)
    
    # 실행이 종료되면 노드를 명시적으로 파괴 (선택 사항)
    minimal_publisher.destroy_node()
    
    # ROS 2 종료
    rclpy.shutdown()

# 엔트리 포인트: 이 파일이 메인 프로그램으로 실행될 때만 main() 호출
if __name__ == '__main__':
    main()