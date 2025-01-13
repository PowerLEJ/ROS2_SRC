import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion # 쿼터니언 -> 오일러 각 변환
from ar_track.move_tb3 import MoveTB3 # TurtleBot3 이동을 제어하는 사용자 정의 클래스

# 타겟 마커 ID를 명령줄 인수로 전달받음
TARGET_ID = int(sys.argv[1]) # argv[1]: 목표 마커 ID

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

# angle(회전 각도)에 대해 90%만 회전하도록 설정
# 이유: 로봇이 회전을 할 때, 실제 환경에서 관성이나 바퀴 슬립 등의 물리적 요인으로 인해 초과 회전이 발생할 수 있습니다. 이를 방지하기 위해 약간 부족하게 회전하도록 보정한 값
FIRST_ROTATION_SCALING_POSITIVE = 0.9

# 음수 방향으로 회전할 때 97%만 회전하도록 설정
# 이유: 양수 방향과 음수 방향의 회전 특성이 다를 수 있으므로, 음수 방향의 초과 회전을 줄이기 위해 약간 다른 값을 사용한 것
FIRST_ROTATION_SCALING_NEGATIVE = 0.97

# 2차 회전 시 목표 각도 R의 87.5%만 회전하도록 설정
# 이유: 첫 번째 회전 후 정렬이 대체로 이루어진 상태에서, 마무리 회전을 더 정밀하게 수행하기 위한 값으로 보입니다. 이 과정에서 지나친 회전을 방지하기 위해 보정된 값
SECOND_ROTATION_SCALING = 0.875

# 거리 계산에 대해 112.5%를 곱해 직진 거리를 보정
# 이유: 로봇이 직진할 때, 센서 데이터나 마커 위치 정보가 완벽히 정확하지 않을 수 있습니다. 이를 보완하기 위해 조금 더 이동하도록 설정한 값
STRAIGHT_DISTANCE_SCALING = 1.125


class TrackMarker(Node):
    """   
                                                    ////////////| ar_marker |////////////
            y                      z                --------+---------+---------+--------
            ^  x                   ^                        |     R-0/|\R-0    R|
            | /                    |                        |       /0|0\       |
     marker |/                     | robot                  |      /  |  \      |
            +------> z    x <------+                        |     /   |   \     |
                                  /                         |  dist   |  dist   |
                                 /                          |   /     |     \   |
                                y                           |  /      |      \  |
                                                            | /       |       \0|
                                                            |/R-0    R|R    R-0\|
    pose.x = position.z                             (0 < O) x---------+---------x (0 > O)
    pose.y = position.x              [0]roll    (pos.x > O) ^                   ^ (pos.x < O)
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose  = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
            
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_lift = self.create_publisher(String, '/lift__msg', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)
        
        self.pose = Pose()
        self.tw   = Twist()
        self.tb3  = MoveTB3()
        self.lift_msg = String()
        
        self.theta   = 0.0
        self.dir     = 0
        self.th_ref  = 0.0
        self.z_ref   = 0.0
        self.cnt_sec = 0
        self.current_theta = 0.0  # 현재 로봇의 회전 각도
        
        self.target_found = False
        
        
    def get_marker_pose_(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        if len(msg.marker_ids) == 0:    # no marker found
            self.target_found = False
        
        else: # if len(msg.marker_ids) != 0: # marker found at least 1EA
        
            for i in range(len(msg.marker_ids)):
            
                if msg.marker_ids[i] == TARGET_ID:  # target marker found
                    if self.target_found == False:
                        self.target_found = True                        
                    self.pose  = msg.poses[i]
                    self.theta = self.get_theta(self.pose)
                    self.current_theta = self.theta  # 현재 로봇의 각도도 업데이트
                else:
                    self.target_found = False            
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta
    
    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1    
        
    def pub_lift_msg(self, lift_msg):
        msg = String()
        msg.data = lift_msg
        self.pub_lift.publish(msg)
    
    def stop_move(self):
        self.tw.linear.z = self.tw.angular.z = 0.0
        self.pub_tw.publish(self.tw)     





    def rotate(self, angular_speed, duration):
        """로봇 회전"""
        self.tw.angular.z = angular_speed
        start_time = self.cnt_sec
        while self.cnt_sec - start_time < duration:
            if self.target_found:  # 마커를 찾으면 즉시 종료
                self.stop_move()
                self.get_logger().info("Marker found during rotation!")
                return
            self.pub_tw.publish(self.tw)
            rclpy.spin_once(self)

    def rotate_until_marker_found(self, angular_speed):
        """마커를 발견할 때까지 회전"""
        self.tw.angular.z = angular_speed
        self.get_logger().info("Rotating to find marker...")
        while not self.target_found:  # 마커를 찾을 때까지 루프
            self.pub_tw.publish(self.tw)
            rclpy.spin_once(self)
        self.stop_move()
        self.get_logger().info("Marker found! Stopping rotation.")

    def move_straight(self, linear_speed, distance):
        """로봇 직진"""
        duration = distance / abs(linear_speed)
        self.tw.linear.x = linear_speed
        start_time = self.cnt_sec
        while self.cnt_sec - start_time < duration:
            self.pub_tw.publish(self.tw)
            rclpy.spin_once(self)
        self.stop_move()

    def move_towards_marker(self):
        """마커로 향해 이동 (각도 및 거리 기반)"""
        if self.pose:
            # 마커와의 각도 계산
            angle_to_marker = self.theta  # 이미 계산된 theta 사용
            print("Angle to marker:", angle_to_marker)

            distance_to_marker = sqrt(self.pose.position.x**2 + self.pose.position.y**2)
            print("Distance to marker:", distance_to_marker)

            # 마커의 방향으로 회전 (각도를 맞춘 후)
            self.rotate_towards_marker(angle_to_marker)

            # x 방향으로 이동
            distance_x = abs(self.pose.position.x)
            print(f"Moving in x direction: {distance_x}m")
            self.move_straight(linear_speed=0.05, distance=distance_x)

            # y 방향으로 이동
            distance_y = abs(self.pose.position.y)
            print(f"Moving in y direction: {distance_y}m")
            self.move_straight(linear_speed=0.05, distance=distance_y)

            print("Reached marker position!")

            

    def rotate_towards_marker(self, target_angle):
        """타겟 각도 방향으로 회전"""
        current_angle = self.current_theta  # 현재 로봇의 회전 각도 얻기
        angle_diff = target_angle - current_angle

        print("Current angle:", current_angle)
        print("Target angle:", target_angle)
        print("Angle difference:", angle_diff)

        # 각도 차이를 -pi ~ pi 범위로 조정
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        # 각도에 따라 회전
        angular_speed = 0.1 if angle_diff > 0 else -0.1
        self.tw.angular.z = angular_speed

        while abs(angle_diff) > 0.05:  # 작은 각도 차이는 무시
            self.pub_tw.publish(self.tw)
            rclpy.spin_once(self)

            current_angle = self.current_theta  # 현재 각도 계속 업데이트
            angle_diff = target_angle - current_angle
            
            # 각도 차이를 -pi ~ pi 범위로 다시 조정
            if angle_diff > pi:
                angle_diff -= 2 * pi
            elif angle_diff < -pi:
                angle_diff += 2 * pi

        print("스탑 안하나")
        self.stop_move() 


    def move_to_marker(self):
        """마커를 향해 정확하게 이동하기 위한 함수"""
        if self.pose:
            # 마커와의 각도 계산
            angle_to_marker = self.theta  # 이미 계산된 theta 사용
            print("Angle to marker:", angle_to_marker)

            distance_to_marker = sqrt(self.pose.position.x**2 + self.pose.position.y**2)
            print("Distance to marker:", distance_to_marker)

            # 마커와의 거리가 너무 가까운 경우
            if distance_to_marker < LIFTING_DISTANCE:
                print("Reached close enough to the marker, starting lifting.")
                self.pub_lift_msg("lift")
                return

            # 마커의 방향으로 회전 (각도를 맞춘 후)
            self.rotate_towards_marker(angle_to_marker)

            # x 방향으로 이동
            distance_x = abs(self.pose.position.x)
            print(f"Moving in x direction: {distance_x}m")
            self.move_straight(linear_speed=0.05, distance=distance_x)

            # y 방향으로 이동
            distance_y = abs(self.pose.position.y)
            print(f"Moving in y direction: {distance_y}m")
            self.move_straight(linear_speed=0.05, distance=distance_y)

            # 목표 지점에 도달했을 때
            if abs(distance_to_marker) <= X_ALIGNMENT_TOLERANCE:
                print("Target reached! Stopping movement.")
                self.stop_move()
                return

            print("Move to marker completed.")
        else:
            print("No marker detected.")

        
        
def main(args=None):

    rclpy.init(args=args)
    node = TrackMarker()
    
    node.tw.angular.z = 0.5 * ANG_SPEED
    
    try:    
        while rclpy.ok():
            if node.theta != 0.0:   break   # this means target marker found
            node.pub_tw.publish(node.tw)
            rclpy.spin_once(node, timeout_sec=0.1)
        
        node.stop_move()
        print("\n----- 1_target marker found!\n") ###########################
        



        # 2. 목표 위치 정렬 및 이동
        node.get_logger().info("Aligning with marker...")
        node.move_to_marker()

        # 3. Lift 동작 수행
        node.get_logger().info("Performing lift operation...")
        node.pub_lift_msg("lift_up")
        node.cnt_sec = 0
        while node.cnt_sec < 10:  # 10초 대기
            rclpy.spin_once(node)

        # 4. 원래 위치로 복귀
        node.get_logger().info("Returning to start position...")
        node.move_straight(linear_speed=-0.05, distance=0.2)  # 뒤로 이동
        node.rotate(angular_speed=-0.1, duration=3)  # 반대 방향 회전
        node.pub_lift_msg("lift_down")
        node.get_logger().info("Task complete.")




        
        
        # sys.exit(1)
        # rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

