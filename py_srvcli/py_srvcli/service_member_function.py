from example_interfaces.srv import AddTwoInts  # AddTwoInts 서비스 메시지 타입 임포트

import rclpy  # ROS 2 Python 클라이언트 라이브러리 임포트
from rclpy.node import Node  # ROS 2 노드를 만들기 위한 Node 클래스 임포트


# AddTwoInts 서비스를 제공하는 서비스 서버 클래스 정의
class MinimalService(Node):

    # 생성자, 서비스 서버를 초기화
    def __init__(self):
        super().__init__('minimal_service')  # 부모 클래스(Node) 생성자 호출, 고유한 노드 이름 설정
        # 'add_two_ints' 서비스 생성, 이 서비스는 AddTwoInts 타입의 요청과 응답을 처리
        # 서비스가 호출되면 add_two_ints_callback 함수가 호출됩니다.
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    # AddTwoInts 서비스 요청을 처리하는 콜백 함수
    def add_two_ints_callback(self, request, response):
        # 요청으로 받은 a와 b를 더하여 응답 객체의 sum 필드에 저장
        response.sum = request.a + request.b
        
        # 요청 받은 값을 로그로 출력
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        # 응답 객체를 반환하여 클라이언트에 결과를 전달
        return response


# 메인 함수, ROS 2 시스템 초기화 및 서비스 서버 실행
def main(args=None):
    rclpy.init(args=args)  # ROS 2 Python 클라이언트 라이브러리 초기화

    # MinimalService 서비스 서버 인스턴스 생성
    minimal_service = MinimalService()

    # 서비스가 요청을 받을 수 있도록 대기
    rclpy.spin(minimal_service)

    # 노드 종료 후 rclpy.shutdown() 호출하여 ROS 2 클라이언트 라이브러리 종료
    rclpy.shutdown()


# 이 파일이 직접 실행될 경우 main 함수 호출
if __name__ == '__main__':
    main()
