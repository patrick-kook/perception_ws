#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import tf
from std_msgs.msg import Bool


class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publisher for Ackermann drive messages
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # Publisher for visualization marker
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)

        # Subscribers for waypoints, scaled waypoints, shrunk waypoints, odometry, and obstacle
        rospy.Subscriber("/waypoints", PoseArray, self.waypoints_callback)
        rospy.Subscriber("/scaled_waypoints", PoseArray, self.scaled_waypoints_callback)
        # rospy.Subscriber("/shrunk_waypoints", PoseArray, self.shrunk_waypoints_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_detected", Bool, self.obstacle_callback)

        # Parameters
        self.waypoints = None
        self.obs_waypoints = None
        self.standard_wapoints = None
        self.colors = None
        self.obstacle_detected = False  # 장애물 탐지 여부
        self.base_lookahead_distance = 1.5  # 기본 lookahead 거리
        self.min_lookahead_distance = 0  # 최소 lookahead 거리
        # self.max_lookahead_distance = 3.0  # 최대 lookahead 거리
        self.speed_factor = 0.7  # 속도에 따른 조정 인자
        self.wheelbase_length = 0.3302  # 차량 휠베이스 길이 (미터)
        self.current_pose = None
        self.current_speed = 0
        self.angle_threshold = np.pi / 4  # 전방 웨이포인트 허용 각도 범위 (45도)
        self.yaw = 0.0  # 차량의 현재 방위각
        self.target_point = None

    def waypoints_callback(self, msg):
        self.waypoints = np.array(
            [[pose.position.x, pose.position.y] for pose in msg.poses]
        )
        self.current_waypoints = self.waypoints  # 초기 경로는 원래 경로로 설정

    def scaled_waypoints_callback(self, msg):
        self.scaled_waypoints = np.array(
            [[pose.position.x, pose.position.y] for pose in msg.poses]
        )

    def shrunk_waypoints_callback(self, msg):
        self.shrunk_waypoints = np.array(
            [[pose.position.x, pose.position.y] for pose in msg.poses]
        )

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

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data  # 장애물 탐지 여부 업데이트

    # Lookahead 거리에 기반하여 목표 지점을 계산하는 함수
    def get_target_point(self, lookahead_distance):
        if self.waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        target_point = None
        target_color = None

        # 웨이포인트들 사이에서 lookahead 거리에 맞는 가장 가까운 포인트를 찾음
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

            # 웨이포인트가 차량 앞에 있는지 확인
            if dist < lookahead_distance:
                continue

            if dist < closest_dist:
                heading = np.arctan2(
                    point[1] - self.current_pose.position.y,
                    point[0] - self.current_pose.position.x,
                )
                angle_diff = abs(
                    np.arctan2(np.sin(heading - self.yaw), np.cos(heading - self.yaw))
                )

                if angle_diff <= self.angle_threshold:
                    closest_dist = dist
                    target_point = point
                    target_color = self.colors[idx]

        # 타겟 포인트 시각화
        if target_point is not None:
            self.publish_marker(target_point)

        return target_point, target_color

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD  # 항상 새로운 점으로 업데이트
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

    def compute_steering_angle(self, target_point):
        # Compute the steering angle using the Pure Pursuit algorithm
        if target_point is None or self.current_pose is None:
            return 0.0

        # 차량의 현재 위치와 목표 지점 사이의 각도를 계산
        alpha = (
            np.arctan2(
                target_point[1] - self.current_pose.position.y,
                target_point[0] - self.current_pose.position.x,
            )
            - self.yaw
        )

        steering_angle = np.arctan(
            2 * self.wheelbase_length * np.sin(alpha) / self.lookahead_distance
        )

        return steering_angle

    def run(self):
        while not rospy.is_shutdown():
            # 장애물이 탐지되었는지 여부에 따라 경로 변경
            if self.obstacle_detected:
                # 장애물이 탐지되면 경로 변경 (축소 또는 확대된 경로로 전환)
                self.lookahead_distance = 1.5   
                if self.shrunk_waypoints is not None:
                    self.current_waypoints = self.shrunk_waypoints
                elif self.scaled_waypoints is not None:
                    self.current_waypoints = self.scaled_waypoints
            else:
                # 장애물이 없으면 원래 경로로 복귀
                self.current_waypoints = self.waypoints
                self.lookahead_distance = self.base_lookahead_distance

            # 현재 속도를 기반으로 lookahead distance를 동적으로 설정
            # self.lookahead_distance = self.base_lookahead_distance

            target_point = self.get_target_point(self.lookahead_distance)
            steering_angle = self.compute_steering_angle(target_point)

            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = 0
            if steering_angle > 0.25 or steering_angle < -0.25:
                drive_msg.drive.speed = 2
            else:
                drive_msg.drive.speed = 4

            self.drive_pub.publish(drive_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
