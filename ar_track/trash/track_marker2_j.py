import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion  # 쿼터니언 -> 오일러 각 변환
from ar_track.move_tb3 import MoveTB3  # TurtleBot3 이동을 제어하는 사용자 정의 클래스

# 타겟 마커 ID를 명령줄 인수로 전달받음
TARGET_ID = int(sys.argv[1])  # argv[1]: 목표 마커 ID

# TurtleBot3의 최대 속도 설정
MAX_LIN_SPEED = 0.22  # 최대 선형 속도 (m/s)
MAX_ANG_SPEED = 2.84  # 최대 각속도 (rad/s)

# 기본 이동 속도 (최대 속도의 7.5%)
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

# 90도 (라디안 값)
R = pi / 2 # radians(90) # 1.5708



# 허용 오차 범위
# 마커의 x 좌표(pose.position.x)가 -0.0025에서 0.0025 사이에 있으면 목표 위치에 정확히 정렬되었다고 판단
# 이는 로봇이 마커에 대해 얼마나 정밀하게 정렬되어야 하는지를 결정하는 허용 오차 값
X_ALIGNMENT_TOLERANCE = 0.0025

# 속도 설정 (각속도)
# ANG_SPEED의 비율로 설정된 각속도 값
# ANG_SPEED의 7.5% 정도로 천천히 회전하여 목표 위치를 정밀하게 조정하려는 의도
# 속도를 낮추면 정밀도가 높아지고, 오차가 줄어듦
FINE_ANGULAR_SPEED = 0.075 * ANG_SPEED  

# 리프팅 기준 거리
# 로봇이 마커에 충분히 가까이 접근해야 리프팅을 할 수 있습니다. 이 값은 리프팅을 위한 목표 거리. 마커와 로봇 사이의 최소 거리를 정의
LIFTING_DISTANCE = 0.185


# 회전 및 거리 보정 비율

FIRST_ROTATION_SCALING = 0.97

# angle(회전 각도)에 대해 90%만 회전하도록 설정
# 이유: 로봇이 회전을 할 때, 실제 환경에서 관성이나 바퀴 슬립 등의 물리적 요인으로 인해 초과 회전이 발생할 수 있습니다. 이를 방지하기 위해 약간 부족하게 회전하도록 보정한 값
# FIRST_ROTATION_SCALING_POSITIVE = 0.9

# 음수 방향으로 회전할 때 97%만 회전하도록 설정
# 이유: 양수 방향과 음수 방향의 회전 특성이 다를 수 있으므로, 음수 방향의 초과 회전을 줄이기 위해 약간 다른 값을 사용한 것
# FIRST_ROTATION_SCALING_NEGATIVE = 0.97

# 2차 회전 시 목표 각도 R의 87.5%만 회전하도록 설정
# 이유: 첫 번째 회전 후 정렬이 대체로 이루어진 상태에서, 마무리 회전을 더 정밀하게 수행하기 위한 값으로 보입니다. 이 과정에서 지나친 회전을 방지하기 위해 보정된 값
SECOND_ROTATION_SCALING = 0.875

# 거리 계산에 대해 112.5%를 곱해 직진 거리를 보정
# 이유: 로봇이 직진할 때, 센서 데이터나 마커 위치 정보가 완벽히 정확하지 않을 수 있습니다. 이를 보완하기 위해 조금 더 이동하도록 설정한 값
STRAIGHT_DISTANCE_SCALING = 1.125



class TrackMarker(Node):
    """
    ArUco 마커를 탐지하고 추적하는 ROS 2 노드 클래스
    """
    def __init__(self):
        super().__init__('track_marker')  # 노드 이름 설정
        qos_profile = QoSProfile(depth=10)  # QoS 설정
        
        # ArUco 마커 구독자 생성
        self.sub_ar_pose = self.create_subscription(
            ArucoMarkers,           # 메시지 타입
            'aruco_markers',        # 토픽 이름
            self.get_marker_pose_,  # 콜백 함수
            qos_profile)

        # 이동 명령 퍼블리셔 생성
        self.pub_tw = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        # 리프팅 명령 퍼블리셔 생성
        self.pub_lift = self.create_publisher(String, '/lift__msg', qos_profile)
        
        # 초 단위 카운터 타이머 생성
        self.timer = self.create_timer(1, self.count_sec)

        # 로봇의 상태를 저장하는 변수들 초기화
        self.pose = Pose()  # 현재 마커의 자세
        self.tw = Twist()   # 이동 명령 메시지
        self.tb3 = MoveTB3()  # TurtleBot3 이동 제어 객체
        self.lift_msg = String()  # 리프팅 메시지

        self.theta = 0.0  # 마커의 현재 각도
        self.dir = 0  # 방향 정보
        self.th_ref = 0.0  # 기준 각도
        self.z_ref = 0.0  # 기준 거리
        self.cnt_sec = 0  # 시간 카운터

        self.target_found = False  # 목표 마커 발견 여부 플래그

    def get_marker_pose_(self, msg):
        """
        ArUco 마커의 위치와 자세를 처리하는 콜백 함수
        """
        if len(msg.marker_ids) == 0:  # 마커를 발견하지 못한 경우
            self.target_found = False
        else:  # 하나 이상의 마커를 발견한 경우
            for i in range(len(msg.marker_ids)):
                if msg.marker_ids[i] == TARGET_ID:  # 목표 마커를 발견한 경우
                    if not self.target_found:
                        self.target_found = True
                    self.pose = msg.poses[i]  # 마커의 자세 저장
                    self.theta = self.get_theta(self.pose)  # 마커의 각도 계산
                else:
                    self.target_found = False

    def get_theta(self, msg):
        """
        마커의 쿼터니언 데이터를 오일러 각도로 변환하고 피치(각도)를 반환
        """
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]  # 피치 각도
        return theta

    def count_sec(self):
        """
        초 단위 카운터 증가 (리프팅 시간 측정에 사용)
        """
        self.cnt_sec += 1

    def pub_lift_msg(self, lift_msg):
        """
        리프팅 메시지 발행
        """
        msg = String()
        msg.data = lift_msg
        self.pub_lift.publish(msg)

    def stop_move(self):
        """
        로봇을 정지시키는 함수
        """
        self.tw.linear.z = self.tw.angular.z = 0.0
        self.pub_tw.publish(self.tw)


def main(args=None):
    """
    로봇의 전체 동작 흐름 제어
    """
    rclpy.init(args=args)
    node = TrackMarker()

    # 초기 회전으로 마커 탐색
    node.tw.angular.z = 0.5 * ANG_SPEED
    try:
        while rclpy.ok():
            if node.theta != 0.0:  # 목표 마커를 발견하면 루프 종료
                break
            node.pub_tw.publish(node.tw)
            rclpy.spin_once(node, timeout_sec=0.1)

        node.stop_move()  # 회전 중지
        print("\n----- 1_target marker found!\n")

        # x 좌표 기준으로 정렬
        while node.pose.position.x < -0.0175 or node.pose.position.x > 0.0175:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose.position.x < -0.0155:
                node.tw.angular.z = 0.175 * ANG_SPEED
            else:
                node.tw.angular.z = -0.125 * ANG_SPEED
            node.pub_tw.publish(node.tw)

        node.stop_move()
        print("\n----- 2_arrived reference position!\n")

        # 기준 각도 및 거리 설정
        node.th_ref = node.theta
        node.z_ref = node.pose.position.z
        node.dir = 1 if node.th_ref >= 0 else -1

        # 첫 번째 회전
        angle = R - node.th_ref
        if angle > R:
            angle = pi - angle
        if node.th_ref > radians(10):
            node.tb3.rotate(angle * FIRST_ROTATION_SCALING)
        elif node.th_ref < radians(-10):
            node.tb3.rotate(-angle * FIRST_ROTATION_SCALING)
        print("\n----- 3_1st rotation finished!\n")

        # 마커 앞으로 이동
        dist1 = abs(node.z_ref * sin(node.th_ref) * STRAIGHT_DISTANCE_SCALING)
        node.tb3.straight(dist1)
        print("\n----- 4_move to front of marker end!\n")

        # 두 번째 회전
        if node.th_ref > radians(10):
            node.tb3.rotate(-R * SECOND_ROTATION_SCALING)
        elif node.th_ref < -radians(10):
            node.tb3.rotate(R * SECOND_ROTATION_SCALING)
        print("\n----- 5_2nd rotation finished!\n")

        # 리프팅 위치로 이동
        while node.pose.position.x < -X_ALIGNMENT_TOLERANCE or node.pose.position.x > X_ALIGNMENT_TOLERANCE:
            if node.pose.position.x < -X_ALIGNMENT_TOLERANCE:
                node.tw.angular.z = FINE_ANGULAR_SPEED
            elif node.pose.position.x > X_ALIGNMENT_TOLERANCE:
                node.tw.angular.z = -FINE_ANGULAR_SPEED
            node.pub_tw.publish(node.tw)
        dist2 = node.pose.position.z - LIFTING_DISTANCE
        node.tb3.straight(dist2)
        print("\n----- 6_arrived lifting position!\n")

        # 리프팅 및 로딩
        node.pub_lift_msg("lift_up")
        duration = node.cnt_sec + 10
        while node.cnt_sec < duration:
            rclpy.spin_once(node, timeout_sec=1.0)
        print("\n----- 7_finished loading!\n")

        # 되돌아가기
        node.tb3.straight(-dist2)
        node.tb3.rotate(R * node.dir)
        node.tb3.straight(-dist1)
        print("\n----- 8_arrived starting point!\n")

        # 리프팅 해제 및 종료
        node.pub_lift_msg("lift_down")
        duration = node.cnt_sec + 8
        while node.cnt_sec < duration:
            rclpy.spin_once(node, timeout_sec=1.0)
        print("\n----- 9_finished unloading!\n")

        sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
