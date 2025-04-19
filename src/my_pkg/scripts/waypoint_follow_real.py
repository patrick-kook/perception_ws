import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
import tf
from std_msgs.msg import Bool


class PurePursuit:

    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # 타겟 포인트 히스토리를 저장할 리스트
        self.target_point_history = []
        self.history_size = 10  # N개의 이전 타겟 포인트를 유지할 크기

        # Publisher for Ackermann drive messages
        #self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        #퍼블리셔 정의
        self.steer_pub = rospy.Publisher(
            "/commands/servo/position", Float64, queue_size=10
        )
        self.speed_pub = rospy.Publisher(
            "/commands/motor/duty_cycle", Float64, queue_size=10
        )

        # 시각화 마커 퍼블리셔
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)

        # 웨이포인트와 오도메트리 구독자
        rospy.Subscriber(
            "/global_path/optimal_trajectory", Marker, self.waypoints_callback
        )
        rospy.Subscriber("/obs_waypoints_marker", Marker, self.obs_waypoints_callback)
        rospy.Subscriber("/stan_waypoints_marker", Marker, self.stan_waypoints_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)

        # 장애물 탐지 구독자
        rospy.Subscriber("/obstacle_detected", Bool, self.obstacle_callback)

        # 매개변수 초기화
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

    # 웨이포인트 콜백
    def waypoints_callback(self, msg):
        self.waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    # 장애물 웨이포인트 콜백
    def obs_waypoints_callback(self, msg):
        self.obs_waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    def stan_waypoints_callback(self, msg):
        self.stan_waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    # 오도메트리 콜백
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

    # 장애물 탐지 콜백
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data  # 장애물 탐지 여부 업데이트

    # 색상에 따른 lookahead 거리 계산
    # def get_closest_waypoint(self):
    #     """
    #     Calculate the index of the closest waypoint to the current vehicle position.
    #     Returns:
    #         curr_idx (int): closest waypoint index
    #     """
    #     if self.waypoints is None or self.current_pose is None:
    #         return None

    #     closest_dist = float('inf')
    #     curr_idx = None

    #     # 차량의 현재 위치 가져오기
    #     current_x = self.current_pose.position.x
    #     current_y = self.current_pose.position.y

    #     # 각 웨이포인트에 대해 거리를 계산
    #     for i, waypoint in enumerate(self.waypoints):
    #         dist = np.linalg.norm(np.array([waypoint[0] - current_x, waypoint[1] - current_y]))
    #         if dist < closest_dist:
    #             closest_dist = dist
    #             curr_idx = i

    #     return curr_idx

    # def compute_lookahead_distance(self, prev_target_point, new_target_point):
    #     """
    #     This method calculates the lookahead distance based on yaw difference
    #     between the previous and new target points.

    #     Args:
    #         prev_target_point: The previous target point [x, y].
    #         new_target_point: The newly selected target point [x, y].
    #     Returns:
    #         lookahead_dist (float): The calculated lookahead distance.
    #     """
    #     L = self.base_lookahead_distance  # 기본 설정된 lookahead 거리
    #     slope = 0.3  # lookahead 거리를 조정하는 감쇠율

    #     # 현재 위치와 이전 목표 지점 간의 yaw 계산
    #     yaw_before = self.compute_yaw(prev_target_point)

    #     # 현재 위치와 새롭게 선택된 목표 지점 간의 yaw 계산
    #     yaw_after = self.compute_yaw(new_target_point)

    #     # yaw 차이 계산
    #     yaw_diff = abs(yaw_after - yaw_before)

    #     # yaw 차이를 [-π, π] 범위로 조정
    #     if yaw_diff > np.pi:
    #         yaw_diff -= 2 * np.pi
    #     if yaw_diff < -np.pi:
    #         yaw_diff += 2 * np.pi
    #     yaw_diff = abs(yaw_diff)

    #     # yaw 변화에 따라 lookahead 거리 조정
    #     if yaw_diff > np.pi / 2:
    #         yaw_diff = np.pi / 2
    #     L = max(
    #         self.min_lookahead_distance, L * (np.pi / 2 - yaw_diff * slope) / (np.pi / 2)
    #     )

    #     return L
    def compute_lookahead_distance(self, target_color):
        """
        This method calculates the lookahead distance based on the target point's color.

        Args:
            target_color: The color associated with the target point [r, g, b, a].
        Returns:
            lookahead_dist (float): The calculated lookahead distance based on color.
        """
        if target_color is None:
            return self.base_lookahead_distance

        red = target_color[0]
        green = target_color[1]

        # Lookahead distance is influenced by the green and red channels.
        # Green increases the lookahead distance, and red decreases it.
        lookahead_distance = self.base_lookahead_distance + 0.5 * green - 1 * red

        # Ensure the lookahead distance is within the specified range
        lookahead_distance = max(self.min_lookahead_distance, lookahead_distance)

        return lookahead_distance

    def compute_yaw(self, waypoint):
        """
        Compute yaw (heading angle) between waypoints for curvature calculation
        Args:
            waypoint: A specific waypoint [x, y] on the path
        Returns:
            yaw angle in radians
        """
        dx = waypoint[0] - self.current_pose.position.x
        dy = waypoint[1] - self.current_pose.position.y
        return np.arctan2(dy, dx)

    # 타겟 포인트를 선택하는 함수

    def get_target_point(self, lookahead_distance):
        current_waypoints = (
            self.obs_waypoints
            if self.obstacle_detected
            else (
                self.waypoints or self.standard_wapoints
                if self.obstacle_detected
                else self.waypoints
            )
        )
        if current_waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        target_point = None
        target_color = None
        best_match_diff = float("inf")

        # 웨이포인트들 사이에서 lookahead 거리 이하 중 가장 가까운 포인트를 찾음
        for idx, point in enumerate(current_waypoints):
            dist = np.linalg.norm(
                np.array(
                    [
                        point[0] - self.current_pose.position.x,
                        point[1] - self.current_pose.position.y,
                    ]
                )
            )

            if dist <= lookahead_distance:
                dist_diff = abs(lookahead_distance - dist)

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
                        best_match_diff = dist_diff
                        closest_dist = dist
                        target_point = point

                        # 색상 인덱스가 범위를 벗어나지 않도록 체크
                        if idx < len(self.colors):
                            target_color = self.colors[idx]

        if target_point is not None:
            self.publish_marker(target_point)

        return target_point, target_color

    # 마커 시각화
    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
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

        alpha = (
            np.arctan2(
                target_point[1] - self.current_pose.position.y,
                target_point[0] - self.current_pose.position.x,
            )
            - self.yaw
        )

        steering_angle = np.arctan(
            2 * self.wheelbase_length * np.sin(alpha) / lookahead_distance
        )

        return steering_angle + 0.5

    # 속도 계산 함수
    def compute_speed(self, target_color):
        if target_color is None:
            return 2.0

        red = target_color[0]
        green = target_color[1]

        speed = 2.5 + 1.0 * green - 2 * red

        return speed

    # 메인 실행 함수

    def run(self):
        while not rospy.is_shutdown():
            # 기본 lookahead 거리를 먼저 색깔에 따라 계산
            lookahead_distance = self.base_lookahead_distance  # 초기 기본값

            # 타겟 포인트 및 색상을 업데이트 (여기서 lookahead_distance 사용)
            self.target_point, target_color = self.get_target_point(lookahead_distance)

            if self.target_point is not None:
                # 타겟 포인트의 색깔로 lookahead 거리를 재계산
                lookahead_distance = self.compute_lookahead_distance(target_color)
                # 룩어헤드 거리를 기반으로 다시 타겟 포인트 업데이트
                self.target_point, target_color = self.get_target_point(
                    lookahead_distance
                )

                # 조향각 계산
                steering_angle = self.compute_steering_angle(
                    self.target_point, lookahead_distance
                )

                # 속도 및 조향을 포함한 Ackermann 메시지 생성
                # drive_msg = AckermannDriveStamped()
                # drive_msg.drive.steering_angle = steering_angle
                # drive_msg.drive.speed = self.compute_speed(target_color)
                speed_msg=Float64()
                steer_msg=Float64()

                # 메시지를 퍼블리시
                speed_msg.data=self.compute_speed
                steer_msg.data=steering_angle
                self.speed_pub.publish(speed_msg)
                self.steer_pub.publish(steer_msg)

                # 디버그용으로 현재 타겟 포인트와 룩어헤드 거리를 출력
                rospy.loginfo(f"Current target point: {self.target_point}")
                rospy.loginfo(f"Lookahead distance: {lookahead_distance}")
            else:
                rospy.logwarn("No target point available, skipping control update.")

            # 루프 딜레이 설정
            self.rate.sleep()

    # def run(self):
    #     prev_target_point = None  # 이전 타겟 포인트 초기화

    #     while not rospy.is_shutdown():
    #         if self.target_point is not None:
    #             # 현재 가장 가까운 웨이포인트 인덱스 가져오기
    #             curr_idx = self.get_closest_waypoint()

    #             # 타겟 포인트 및 색상을 업데이트
    #             self.target_point, target_color = self.get_target_point(
    #                 self.base_lookahead_distance
    #             )

    #             # 타겟 포인트 히스토리에 추가
    #             self.target_point_history.append(self.target_point)

    #             # 히스토리가 너무 길어지면 가장 오래된 항목을 제거
    #             if len(self.target_point_history) > self.history_size:
    #                 self.target_point_history.pop(0)

    #             # 10번 전의 타겟 포인트를 prev_target_point로 사용
    #             if len(self.target_point_history) >= 10:
    #                 prev_target_point = self.target_point_history[-8]  # 10번째 전의 타겟
    #             else:
    #                 prev_target_point = self.target_point_history[
    #                     0
    #                 ]  # 아직 충분한 데이터가 없을 때 첫 타겟 사용

    #             # 이전 타겟 포인트가 있으면 그것을 사용하여 룩어헤드 거리 계산
    #             if prev_target_point is not None and self.target_point is not None:
    #                 lookahead_distance = self.compute_lookahead_distance(
    #                     prev_target_point, self.target_point
    #                 )
    #             else:
    #                 rospy.logwarn(
    #                     "Previous target point is None, using base lookahead distance."
    #                 )
    #                 lookahead_distance = self.base_lookahead_distance

    #             # 조향각 계산
    #             steering_angle = self.compute_steering_angle(
    #                 self.target_point, lookahead_distance
    #             )

    #         else:
    #             # 처음에는 target_point가 없으므로 기본 값으로 설정
    #             self.target_point, target_color = self.get_target_point(
    #                 self.base_lookahead_distance
    #             )

    #             # 기본 룩어헤드 거리 사용
    #             lookahead_distance = self.base_lookahead_distance

    #             # 기본 룩어헤드 거리로 조향각 계산
    #             steering_angle = self.compute_steering_angle(
    #                 self.target_point, lookahead_distance
    #             )

    #             # 첫 타겟 포인트를 히스토리에 추가
    #             self.target_point_history.append(self.target_point)

    #         # 속도 및 조향을 포함한 Ackermann 메시지 생성
    #         drive_msg = AckermannDriveStamped()
    #         drive_msg.drive.steering_angle = steering_angle
    #         drive_msg.drive.speed = self.compute_speed(target_color)

    #         # 메시지를 퍼블리시
    #         self.drive_pub.publish(drive_msg)

    #         # 디버그용으로 현재 타겟 포인트와 룩어헤드 거리를 출력
    #         print(f"Current target point: {self.target_point}")
    #         print(f"Previous target point (10 steps ago): {prev_target_point}")
    #         print(f"Lookahead distance: {lookahead_distance}")

    #         # 루프 딜레이 설정
    #         self.rate.sleep()

    #     self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
