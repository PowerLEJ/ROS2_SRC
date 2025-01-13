import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import radians, pi, sin

class TrackMarker(Node):
    def __init__(self):
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        # Subscriber & Publisher
        self.sub_ar_pose = self.create_subscription(
            ArucoMarkers, 'aruco_markers', self.get_marker_pose, qos_profile
        )
        self.pub_tw = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_lift = self.create_publisher(String, '/lift_msg', qos_profile)
        self.timer = self.create_timer(1.0, self.count_sec)
        
        # 변수 초기화
        self.twist = Twist()
        self.target_found = False
        self.marker_pose = None
        self.timer_count = 0

    def get_marker_pose(self, msg):
        """ArUco Marker 데이터 업데이트"""
        if len(msg.marker_ids) > 0:
            for i in range(len(msg.marker_ids)):
                if msg.marker_ids[i] == int(sys.argv[1]):  # 명령줄 인수로 타겟 ID 설정
                    self.target_found = True
                    self.marker_pose = msg.poses[i]
                    self.get_logger().info(f"Marker ID {msg.marker_ids[i]} detected!")
                    break

    def count_sec(self):
        """타이머 증가"""
        self.timer_count += 1

    def stop_movement(self):
        """로봇 정지"""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub_tw.publish(self.twist)

    def rotate(self, angular_speed, duration):
        """로봇 회전"""
        self.twist.angular.z = angular_speed
        start_time = self.timer_count
        while self.timer_count - start_time < duration:
            if self.target_found:  # 마커를 찾으면 즉시 종료
                self.stop_movement()
                self.get_logger().info("Marker found during rotation!")
                return
            self.pub_tw.publish(self.twist)
            rclpy.spin_once(self)


    def rotate_until_marker_found(self, angular_speed):
        """마커를 발견할 때까지 회전"""
        self.twist.angular.z = angular_speed
        self.get_logger().info("Rotating to find marker...")
        while not self.target_found:  # 마커를 찾을 때까지 루프
            self.pub_tw.publish(self.twist)
            rclpy.spin_once(self)
        self.stop_movement()
        self.get_logger().info("Marker found! Stopping rotation.")


    def move_straight(self, linear_speed, distance):
        """로봇 직진"""
        duration = distance / abs(linear_speed)
        self.twist.linear.x = linear_speed
        start_time = self.timer_count
        while self.timer_count - start_time < duration:
            self.pub_tw.publish(self.twist)
            rclpy.spin_once(self)
        self.stop_movement()

    def publish_lift_msg(self, msg):
        """Lift 메시지 퍼블리시"""
        lift_msg = String()
        lift_msg.data = msg
        self.pub_lift.publish(lift_msg)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Error: Marker ID not provided. Usage: python script_name.py <marker_id>")
        sys.exit(1)

    try:
        TARGET_ID = int(sys.argv[1])  # 마커 ID를 정수로 변환
    except ValueError:
        print("Error: Invalid Marker ID. Please provide a valid integer.")
        sys.exit(1)

    node = TrackMarker()

    try:
        # 1. 시계 반대 방향 회전하며 마커 탐색
        node.get_logger().info("Rotating to find marker...")
        # node.rotate(angular_speed=0.07, duration=60)  # 60초 동안 회전
        node.rotate_until_marker_found(angular_speed=0.07) # 찾을 때까지 회전


        if not node.target_found:
            node.get_logger().error("Target marker not found. Exiting...")
            return

        # 2. 목표 위치 정렬 및 이동
        node.get_logger().info("Aligning with marker...")
        node.move_straight(linear_speed=0.05, distance=0.2)  # 마커와 충돌하지 않을 거리

        # 3. Lift 동작 수행
        node.get_logger().info("Performing lift operation...")
        node.publish_lift_msg("lift_up")
        node.timer_count = 0
        while node.timer_count < 10:  # 10초 대기
            rclpy.spin_once(node)

        # 4. 원래 위치로 복귀
        node.get_logger().info("Returning to start position...")
        node.move_straight(linear_speed=-0.05, distance=0.2)  # 뒤로 이동
        node.rotate(angular_speed=-0.1, duration=3)  # 반대 방향 회전
        node.publish_lift_msg("lift_down")
        node.get_logger().info("Task complete.")

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.stop_movement()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
