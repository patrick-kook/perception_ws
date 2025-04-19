import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
import tf
from message_filters import Subscriber, TimeSynchronizer
from matplotlib.path import Path
import pandas as pd
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped

class KalmanFilter:
    def __init__(self, dt, initial_position):
        self.dt = dt
        self.x = np.array([[initial_position[0]], [initial_position[1]], [0], [0]])
        self.A = np.array(
            [[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.P = np.eye(4)
        self.Q = np.array(
            [[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 0.5, 0], [0, 0, 0, 0.5]]
        )
        self.R = np.array([[1.0, 0], [0, 1.0]])

    def predict(self):
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, measurement):
        z = np.array([[measurement[0]], [measurement[1]]])
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))

    def get_state(self):
        return self.x.flatten()


class ObstacleDetection:
    def __init__(self):
        # 장애물 퍼블리셔 (위치와 속도)
        self.obstacle_position_pub = rospy.Publisher(
            "/detected_obstacles", PointStamped, queue_size=50
        )
        self.obstacle_velocity_pub = rospy.Publisher(
            "/detected_obstacle_velocity", Vector3Stamped, queue_size=50
        )
        self.obstacle_detected_pub = rospy.Publisher(
            "/obstacle_detected", Bool, queue_size=10
        )

        lidar_sub = Subscriber("/scan", LaserScan)
        odom_sub = Subscriber("/state_estated/odom", Odometry)
        self.ts = TimeSynchronizer([lidar_sub, odom_sub], 10)
        self.ts.registerCallback(self.synchronized_callback)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_vx = 0.0  # 차량 속도의 x 성분
        self.robot_vy = 0.0  # 차량 속도의 y 성분
        self.yaw = 0.0
        self.target_distance = 0.5
        self.kalman_filters = {}

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_vx = msg.twist.twist.linear.x  # x축 성분
        self.robot_vy = msg.twist.twist.linear.y  # y축 성분
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(orientation_list)

    def is_point_within_angle_limit(self, point):
        """
        차량이 보고 있는 방향을 기준으로 각도 제한 내에 있는지 확인.

        Args:
            point (tuple): LiDAR로부터 탐지된 장애물의 맵 좌표계 위치 (x, y).

        Returns:
            bool: 장애물이 각도 제한 내에 있으면 True.
        """
        # 차량 기준 장애물의 상대 위치 계산
        dx = point[0] - self.robot_x
        dy = point[1] - self.robot_y

        # 장애물의 상대 각도 계산
        angle_to_point = np.arctan2(dy, dx)

        # 차량의 현재 방향과 비교 (yaw)
        angle_diff = abs(
            np.arctan2(np.sin(angle_to_point - self.yaw), np.cos(angle_to_point - self.yaw))
        )

        # 각도 제한 
        angle_limit = np.pi * 0.11
        return angle_diff <= angle_limit

    def transform_to_odom(self, x, y):
        lidar_offset_x = 0.275
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)
        transformed_x = self.robot_x + (x + lidar_offset_x) * cos_yaw - y * sin_yaw
        transformed_y = self.robot_y + (x + lidar_offset_x) * sin_yaw + y * cos_yaw
        return transformed_x, transformed_y


    def lidar_callback(self, scan_data):
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        ranges = scan_data.ranges
        obstacle_detected = False

        for i, r in enumerate(ranges):
            if r < scan_data.range_min or r > scan_data.range_max:
                continue

            angle = angle_min + i * angle_increment
            if -np.pi / 2 <= angle <= np.pi / 2:  # LiDAR의 전방 탐지 범위
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                transformed_x, transformed_y = self.transform_to_odom(x, y)
                lidar_point = (transformed_x, transformed_y)

                # 각도 제한으로 필터링
                if self.is_point_within_angle_limit(lidar_point):
                    distance_to_robot = np.linalg.norm(
                        [transformed_x - self.robot_x, transformed_y - self.robot_y]
                    )
                    if distance_to_robot <= self.target_distance:
                        obstacle_detected = True

                        # 칼만 필터로 위치 추정
                        if lidar_point not in self.kalman_filters:
                            self.kalman_filters[lidar_point] = KalmanFilter(
                                dt=0.1, initial_position=lidar_point
                            )

                        kf = self.kalman_filters[lidar_point]
                        kf.predict()
                        kf.update(lidar_point)
                        state = kf.get_state()
                        filtered_x, filtered_y, vx_rel, vy_rel = state

                        # 장애물 위치 퍼블리시
                        position_msg = PointStamped()
                        position_msg.header.stamp = rospy.Time.now()
                        position_msg.header.frame_id = "map"
                        position_msg.point.x = filtered_x
                        position_msg.point.y = filtered_y
                        position_msg.point.z = 0.0
                        self.obstacle_position_pub.publish(position_msg)

                        # 장애물 속도 퍼블리시
                        velocity_msg = Vector3Stamped()
                        velocity_msg.header.stamp = rospy.Time.now()
                        velocity_msg.header.frame_id = "map"
                        velocity_msg.vector.x = vx_rel
                        velocity_msg.vector.y = vy_rel
                        velocity_msg.vector.z = 0.0
                        self.obstacle_velocity_pub.publish(velocity_msg)

        self.obstacle_detected_pub.publish(Bool(obstacle_detected))

    def synchronized_callback(self, scan_data, odom_data):
        self.odom_callback(odom_data)
        self.lidar_callback(scan_data)

    def run(self):
        rospy.spin()


def obstacle_detection_node():
    rospy.init_node("obstacle_detection_node", anonymous=True)
    node = ObstacleDetection()
    node.run()


if __name__ == "__main__":
    try:
        obstacle_detection_node()
    except rospy.ROSInterruptException:
        pass
