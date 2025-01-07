# ROS 2 라이브러리 임포트
import rclpy                           # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node            # ROS 2의 기본 노드 클래스
from std_msgs.msg import String        # ROS 2 표준 메시지 타입(String)

# 구독자 노드를 정의하는 클래스
class MinimalSubscriber(Node):

    def __init__(self):
        # 부모 클래스(Node)를 초기화하면서 노드 이름을 설정
        super().__init__('minimal_subscriber')  # 노드 이름을 'minimal_subscriber'로 설정
        
        # 구독자 생성
        # - 메시지 타입: String
        # - 토픽 이름: 'hello'
        # - 콜백 함수: self.listener_callback
        # - QoS 설정: 메시지 대기열 크기 10
        self.create_subscription(
            String,                    # 구독할 메시지 타입 (std_msgs.msg.String)
            'hello',                   # 구독할 토픽 이름
            self.listener_callback,    # 메시지를 수신할 때 호출되는 콜백 함수
            10                         # QoS: 메시지 대기열 크기
        )

    # 메시지 수신 시 호출되는 콜백 함수
    def listener_callback(self, msg):
        # 수신한 메시지 내용을 ROS 2 로그로 출력
        # msg.data는 String 메시지의 데이터 필드
        self.get_logger().info('I heard: "%s"' % msg.data)

# 메인 함수
def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    
    # 구독자 노드 인스턴스 생성
    minimal_subscriber = MinimalSubscriber()
    
    # 노드를 실행하여 토픽 메시지를 대기 (구독)
    rclpy.spin(minimal_subscriber)
    
    # 실행이 끝나면 노드를 명시적으로 파괴 (선택 사항)
    minimal_subscriber.destroy_node()
    
    # ROS 2 종료
    rclpy.shutdown()

# 엔트리 포인트: 이 파일이 메인 프로그램으로 실행될 때만 main() 호출
if __name__ == '__main__':
    main()