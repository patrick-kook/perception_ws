import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Bool
import tf


class PurePursuit:

    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # 타겟 포인트 히스토리를 저장할 리스트
        self.target_point_history = []
        self.history_size = 10  # N개의 이전 타겟 포인트를 유지할 크기

        # Publisher for Ackermann drive messages
        # self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # 시각화 마커 퍼블리셔
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.speed_pub = rospy.Publisher("/commands/motor/duty_cycle", Float64, queue_size=10)

        # 웨이포인트와 오도메트리 구독자
        rospy.Subscriber(
            "/global_path/optimal_trajectory", Marker, self.waypoints_callback
        )
        rospy.Subscriber("/global_path/lane_2", Marker, self.obs_waypoints_callback)
        # rospy.Subscriber("/stan_waypoints_marker", Marker, self.stan_waypoints_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)

        # 장애물 탐지 구독자
        rospy.Subscriber("/obstacle_detected", Bool, self.obstacle_callback)

        # 매개변수 초기화

        self.overtaking_wp = np.load(
            "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/1002/overtaking_wp_idx.npy"
        )  # Load overtaking path
        print(self.overtaking_wp)
        self.obstacle_detected = False  # 장애물 탐지 여부
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

    # 오도메트리 콜백
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_speed = 0
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

    # 차량이 overtaking 경로 위에 있는지 확인하는 함수

    def is_on_overtaking_path(self):
        # 차량의 현재 위치 또는 웨이포인트가 없으면 False
        if self.current_pose is None or self.waypoints is None:
            return False

        # 가장 가까운 웨이포인트와 그 거리를 계산
        closest_idx, closest_dist = self.get_closest_waypoint()

        # 가까운 웨이포인트가 None이면 False 반환
        if closest_idx is None:
            return False

        # 가장 가까운 웨이포인트가 오버테이킹 경로에 포함된 경우 True 반환
        if closest_idx in self.overtaking_wp:
            return True

        return False

    def compute_lookahead_distance(self, target_color):
        """
        This method calculates the lookahead distance based on the target point's color.

        Args:
            target_color: The color associated with the target point [r, g, b, a].
        Returns:
            lookahead_dist (float): The calculated lookahead distance based on color.
        """
        if target_color is None:
            lookahead_distance = self.base_lookahead_distance
        else:
            red = target_color[0]
            green = target_color[1]

            # Lookahead distance is influenced by the green and red channels.
            # Green increases the lookahead distance, and red decreases it.
            lookahead_distance = self.base_lookahead_distance + 0.5 * green - 1 * red

        # 장애물이 감지되고 오버테이킹 구간일 때 룩어헤드 거리를 0.8배로 줄임
        if self.obstacle_detected and self.is_on_overtaking_path():
            lookahead_distance *= 0.9

        # Ensure the lookahead distance is within the specified range
        lookahead_distance = max(self.min_lookahead_distance, lookahead_distance)

        return lookahead_distance

    def get_closest_waypoint(self):
        """
        Calculate the index of the closest waypoint to the current vehicle position.
        Returns:
            closest_idx (int): closest waypoint index
            closest_dist (float): distance to the closest waypoint
        """
        if self.waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        closest_idx = None

        # 차량의 현재 위치 가져오기
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # 각 웨이포인트에 대해 거리를 계산
        for i, waypoint in enumerate(self.waypoints):
            dist = np.linalg.norm(
                np.array([waypoint[0] - current_x, waypoint[1] - current_y])
            )
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        return closest_idx, closest_dist

    # 타겟 포인트 선택 함수 (장애물 유무에 따라 경로를 선택)
    def get_target_point(self, lookahead_distance):
        if self.obstacle_detected and self.is_on_overtaking_path():
            current_waypoints = (
                self.obs_waypoints
            )  # 장애물이 있고 overtaking 경로에 있을 때
        else:
            current_waypoints = self.waypoints  # 기본 경로

        if current_waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        target_point = None
        target_color = None
        best_match_diff = float("inf")

        # 웨이포인트 중에서 lookahead 거리 안에 있는 포인트 찾기
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

    # 조향각 계산 함수
    def compute_steering_angle(self, target_point, lookahead_distance):
        if target_point is None:
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

    # 속도 계산 함수 (색상을 기반으로 기본 속도 계산, 장애물 여부에 따라 조정)
    def compute_speed(self, target_color):
        if target_color is None:
            speed = 0.06
        else:
            red = target_color[0]
            green = target_color[1]
            speed = 0.13 + 1 * green - 0.07 * red

        if self.obstacle_detected:
            if self.is_on_overtaking_path():
                return (
                    speed  # 장애물이 있지만 오버테이킹 경로에 있을 경우 기본 속도 유지
                )
            else:
                return max(
                    0.1 * 0.5, 0.1
                )  # 장애물이 있고 오버테이킹 경로에 없을 경우 감속
        return speed  # 장애물이 없으면 기본 속도 유지

    # 실행 함수

    def run(self):
        while not rospy.is_shutdown():
            # 차량 위치에서 가장 가까운 웨이포인트를 찾기
            closest_idx, closest_dist = self.get_closest_waypoint()

            if closest_idx is not None and closest_dist is not None:
                # 가장 가까운 웨이포인트의 색상을 기반으로 lookahead 거리 계산
                target_color = (
                    self.colors[closest_idx] if closest_idx < len(self.colors) else None
                )
                lookahead_distance = self.compute_lookahead_distance(target_color)

                # 그 후 target_point를 선택
                self.target_point, _ = self.get_target_point(lookahead_distance)

                if self.target_point is not None:
                    # 조향각 계산
                    steering_angle = self.compute_steering_angle(
                        self.target_point, lookahead_distance
                    )

                    # # 속도 및 조향을 포함한 Ackermann 메시지 생성
                    # drive_msg = AckermannDriveStamped()
                    # drive_msg.drive.steering_angle = steering_angle
                    # drive_msg.drive.speed = self.compute_speed(target_color)

                    # # 메시지를 퍼블리시
                    # self.drive_pub.publish(drive_msg)
                    steer_msg=Float64()
                    speed_msg = Float64()

                    steer_msg.data = steering_angle
                    speed_msg.data = self.compute_speed(target_color)
                    self.speed_pub.publish(self.speed_msg)
                    self.steer_pub.publish(self.steer_msg)

                    # 디버그용으로 현재 타겟 포인트와 룩어헤드 거리를 출력
                    # rospy.loginfo(f"Current target point: {self.target_point}")
                    # rospy.loginfo(f"Lookahead distance: {lookahead_distance}")
                else:
                    pass
                    # rospy.logwarn("No target point available, skipping control update.")
            else:
                pass
                # rospy.logwarn("No closest waypoint found, skipping control update.")
            print(self.is_on_overtaking_path())
            # 루프 딜레이 설정
            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
