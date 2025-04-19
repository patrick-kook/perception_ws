#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
import tf


class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publisher for Ackermann drive messages
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        #self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        #self.speed_pub = rospy.Publisher("/commands/motor/duty_cycle", Float64, queue_size=10)

        # Publisher for visualization marker
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)

        # Subscribers for waypoints and odometry
        rospy.Subscriber("/waypoints", PoseArray, self.waypoints_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Parameters
        self.waypoints = None
        self.base_lookahead_distance = 1.2  # 기본 lookahead distance
        self.speed_factor = 0.7  # 속도에 따른 조정 인자
        self.wheelbase_length = 0.3302  # meters
        self.current_pose = None
        self.current_speed = 0
        self.angle_threshold = np.pi / 5 * 2  # 전방 웨이포인트 허용 각도 범위 (45도)
        self.max_lookahead_distance = 2.0  # 최대 lookahead 거리
        self.yaw = 0.0  # 차체의 방위각

    def waypoints_callback(self, msg):
        self.waypoints = np.array(
            [[pose.position.x, pose.position.y] for pose in msg.poses]
        )

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        print(self.current_pose)
        # Assuming current_speed should be set from another source or remain zero
        self.current_speed = 0
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(orientation_list)

    def get_target_point(self, lookahead_distance):
        # Find the lookahead point in the waypoints
        if self.waypoints is None or self.current_pose is None:
            return None

        closest_dist = float("inf")
        target_point = None

        for point in self.waypoints:
            # 차량의 현재 위치로부터 각 웨이포인트까지의 거리를 계산합니다
            dist = np.linalg.norm(
                np.array(
                    [
                        point[0] - self.current_pose.position.x,
                        point[1] - self.current_pose.position.y,
                    ]
                )
            )
            if (
                lookahead_distance <= dist <= self.max_lookahead_distance
                and dist < closest_dist
            ):
                # 전방 웨이포인트인지 확인
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

        if target_point is not None:
            self.publish_marker(target_point)

        return target_point

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

        # 차량의 현재 위치와 목표 지점 사이의 각도를 계산합니다
        alpha = (
            np.arctan2(
                target_point[1] - self.current_pose.position.y,
                target_point[0] - self.current_pose.position.x,
            )
            - self.yaw
        )  # 방위각을 고려하여 alpha 값을 계산합니다

        steering_angle = np.arctan(
            2 * self.wheelbase_length * np.sin(alpha) / self.lookahead_distance
        )

        return steering_angle

    def run(self):
        while not rospy.is_shutdown():
            # 현재 속도를 기반으로 lookahead distance를 동적으로 설정
            self.lookahead_distance = self.base_lookahead_distance

            target_point = self.get_target_point(self.lookahead_distance)
            steering_angle = self.compute_steering_angle(target_point)

            # drive_speed = Float64()
            # drive_steer=Float64()
            # drive_steer.data = steering_angle
            # drive_speed.data = 0

            drive=AckermannDriveStamped()
            drive.drive.speed=0
            drive.drive.steering_angle=steering_angle
            if steering_angle > 0.25 or steering_angle < -0.25:
                drive.drive.speed = 1
            else:
                drive.drive.speed = 1

            self.drive_pub.publish(drive)
            # self.steer_pub.publish(drive_steer)
            # self.speed_pub.publish(drive_steer)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
