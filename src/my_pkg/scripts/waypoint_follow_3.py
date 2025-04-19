#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import tf


class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Ackermann 드라이브 메시지를 발행하는 퍼블리셔
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # 시각화 마커를 발행하는 퍼블리셔
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)

        # 웨이포인트와 오도메트리 구독자
        rospy.Subscriber(
            "/waypoints_marker", Marker, self.waypoints_callback
        )
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 매개변수
        self.waypoints = None
        self.colors = None
        self.base_lookahead_distance = 1.2  # 기본 lookahead 거리
        self.min_lookahead_distance = 0.2  # 최소 lookahead 거리
        self.max_lookahead_distance = 3.0  # 최대 lookahead 거리
        self.speed_factor = 0.7  # 속도에 따른 조정 인자
        self.wheelbase_length = 0.3302  # 차량 휠베이스 길이 (미터)
        self.current_pose = None
        self.current_speed = 0
        self.angle_threshold = np.pi / 2  # 전방 웨이포인트 허용 각도 범위 (45도)
        self.yaw = 0.0  # 차량의 현재 방위각

    # 웨이포인트 데이터를 받아오는 콜백 함수
    def waypoints_callback(self, msg):
        self.waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    # 오도메트리 데이터를 받아오는 콜백 함수
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_speed = msg.twist.twist.linear.x
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(orientation_list)

    # 색상에 따라 lookahead 거리를 동적으로 조정하는 함수
    # 색상에 따라 lookahead 거리를 동적으로 조정하는 함수
    def compute_lookahead_distance(self, target_color):
        # lookahead 거리를 색상에 따라 조정 (초록색: 멀리, 빨간색: 가깝게)
        if target_color is None:
            return self.base_lookahead_distance  # 기본값 사용

        # 빨간색 및 초록색 성분 추출
        red = target_color[0]
        green = target_color[1]

        # 초록색일수록 lookahead 거리를 멀리, 빨간색일수록 가까이 설정
        lookahead_distance = (
            self.base_lookahead_distance + green * 0.5 - red * 0.5
        )  # 필요한 경우 이 값을 수정 가능

        # 최소 및 최대 값으로 제한
        lookahead_distance = max(self.min_lookahead_distance, lookahead_distance)
        lookahead_distance = min(self.max_lookahead_distance, lookahead_distance)

        return lookahead_distance

    # Lookahead 거리에 기반하여 목표 지점을 계산하는 함수

    def get_target_point(self, lookahead_distance):
        if self.waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        target_point = None
        target_color = None
        best_match_diff = float(
            "inf"
        )  # lookahead 거리에 가장 근접한 포인트를 찾기 위한 변수

        # 웨이포인트들 사이에서 lookahead 거리 이하 중 가장 가까운 포인트를 찾음
        for idx, point in enumerate(self.waypoints):
            # 차량의 현재 위치로부터 각 웨이포인트까지의 거리를 계산
            dist = np.linalg.norm(
                np.array(
                    [
                        point[0] - self.current_pose.position.x,
                        point[1] - self.current_pose.position.y,
                    ]
                )
            )

            # lookahead_distance 이하의 거리 내에서 lookahead 거리와 가장 가까운 포인트를 선택
            if dist <= lookahead_distance:
                dist_diff = abs(lookahead_distance - dist)

                # lookahead 거리와 가장 근접한 포인트를 선택
                if dist_diff < best_match_diff:
                    heading = np.arctan2(
                        point[1] - self.current_pose.position.y,
                        point[0] - self.current_pose.position.x,
                    )
                    angle_diff = abs(
                        np.arctan2(
                            np.sin(heading - self.yaw), np.cos(heading - self.yaw)
                        )
                    )
                    if angle_diff <= self.angle_threshold:
                        best_match_diff = dist_diff  # 가장 근접한 거리 차이를 저장
                        closest_dist = dist
                        target_point = point
                        target_color = self.colors[idx]

        if target_point is not None:
            self.publish_marker(target_point)

        return target_point, target_color

    # 목표 지점을 시각화하는 함수 (Marker 메시지 사용)
    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD  # 마커를 업데이트
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

    # Pure Pursuit 알고리즘을 사용해 조향각을 계산하는 함수
    def compute_steering_angle(self, target_point, lookahead_distance):
        if target_point is None or self.current_pose is None:
            return 0.0

        # 차량의 현재 위치와 목표 지점 사이의 각도(alpha)를 계산
        alpha = (
            np.arctan2(
                target_point[1] - self.current_pose.position.y,
                target_point[0] - self.current_pose.position.x,
            )
            - self.yaw
        )  # 방위각을 고려하여 alpha 값을 계산

        # Pure Pursuit 공식으로 조향각을 계산 (lookahe  ad_distance를 인자로 사용)
        steering_angle = np.arctan(
            2 * self.wheelbase_length * np.sin(alpha) / lookahead_distance
        )

        return steering_angle

    # 색상 정보에 따라 속도를 계산하는 함수
    def compute_speed(self, target_color):
        # 속도를 빨간색(느리게), 초록색(빠르게)에 따라 조정
        if target_color is None:
            return 5.0  # 기본 속도

        # 빨간색 및 초록색 성분 추출
        red = target_color[0]
        green = target_color[1]

        # 초록색일수록 빠르게, 빨간색일수록 느리게 속도를 조정
        speed = 7.0 + green * 3.0 - red * 6.5  # 필요한 경우 이 값을 수정 가능
        return speed

    # 메인 실행 함수


# 메인 실행 함수
    def run(self):
        while not rospy.is_shutdown():
            # 현재 속도와 컬러 기반으로 lookahead distance 계산
            target_point, target_color = self.get_target_point(self.base_lookahead_distance)

            if target_point is not None:
                # 속도에 따른 lookahead 거리 계산
                lookahead_distance = self.compute_lookahead_distance(target_color)

                # 해당 룩어헤드 거리 내에서 가장 가까운 포인트를 타겟으로 설정
                target_point, target_color = self.get_target_point(lookahead_distance)

                # lookahead_distance를 compute_steering_angle에 전달
                steering_angle = self.compute_steering_angle(
                    target_point, lookahead_distance
                )
                speed = self.compute_speed(target_color)

                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = steering_angle
                drive_msg.drive.speed = speed

                self.drive_pub.publish(drive_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
