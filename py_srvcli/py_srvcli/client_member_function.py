import sys  # 커맨드 라인 인수 접근을 위한 sys 모듈 임포트

# example_interfaces 패키지에서 AddTwoInts 서비스 메시지 타입 임포트
from example_interfaces.srv import AddTwoInts
import rclpy  # ROS 2 Python 클라이언트 라이브러리 임포트
from rclpy.node import Node  # ROS 2 노드를 만들기 위한 Node 클래스 임포트

# AddTwoInts 서비스 클라이언트를 위한 MinimalClientAsync 클래스 정의
class MinimalClientAsync(Node):

    # 생성자, 클라이언트 노드를 초기화
    def __init__(self):
        super().__init__('minimal_client_async')  # 부모 클래스(Node) 생성자 호출, 고유한 노드 이름 설정
        # 'add_two_ints' 서비스와 통신하기 위한 클라이언트 생성
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # 서비스가 사용 가능할 때까지 1초마다 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # AddTwoInts 서비스 요청 객체 생성
        self.req = AddTwoInts.Request()

    # a와 b 두 숫자를 받아서 요청을 보내는 메서드
    def send_request(self, a, b):
        # 요청 객체의 a와 b 필드에 값을 설정
        self.req.a = a
        self.req.b = b
        
        # 서비스를 비동기적으로 호출하고 미래(future) 객체에 결과를 저장
        self.future = self.cli.call_async(self.req)

        # 서비스 응답이 올 때까지 블로킹하여 대기
        rclpy.spin_until_future_complete(self, self.future)

        # 서비스 호출 결과를 반환
        return self.future.result()


# 메인 함수, ROS 2 시스템 초기화 및 서비스 호출
def main(args=None):
    rclpy.init(args=args)  # ROS 2 Python 클라이언트 라이브러리 초기화

    # MinimalClientAsync 클라이언트 노드 인스턴스 생성
    minimal_client = MinimalClientAsync()

    # 커맨드 라인 인수로 받은 두 숫자를 send_request 메서드에 전달하여 서비스 호출
    # sys.argv[1]과 sys.argv[2]는 커맨드 라인에서 입력된 두 숫자
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    # 서비스 호출 결과 로그로 출력 (두 숫자의 합)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    # 서비스 호출이 끝난 후 노드 파괴
    minimal_client.destroy_node()

    # ROS 2 클라이언트 라이브러리 종료
    rclpy.shutdown()


# 이 파일이 직접 실행될 경우 main 함수 호출
if __name__ == '__main__':
    main()
