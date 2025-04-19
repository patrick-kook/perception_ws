import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Bool
from collections import deque
import tf


class PurePursuit:

    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.obstacle_points = deque(maxlen=30)

        self.rate = rospy.Rate(10)  # 10 Hz

        # Publisher for Ackermann drive messages
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # 시각화 마커 퍼블리셔
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)
        self.lookahead_pub=rospy.Publisher("/lookahead_distance",Float64,queue_size=10)

        # 웨이포인트와 오도메트리 구독자
        rospy.Subscriber(
            "/global_path/optimal_trajectory", Marker, self.waypoints_callback
        )
        rospy.Subscriber("/global_path/lane_2", Marker, self.obs_inner_waypoints_callback)
        rospy.Subscriber("/global_path/lane_3", Marker, self.obs_outer_waypoints_callback)
        # rospy.Subscriber("/stan_waypoints_marker", Marker, self.stan_waypoints_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 장애물 탐지 구독자
        rospy.Subscriber("/obstacle_detected", Bool, self.obstacle_callback)
        rospy.Subscriber("/detected_obstacles", PointStamped, self.obstacle_position_callback)

        # 매개변수 초기화

        self.overtaking_wp = np.load(
            "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/1002/overtaking_wp_idx.npy"
        )  # Load overtaking path
        print(self.overtaking_wp)
        self.obstacle_detected = False  # 장애물 탐지 여부
        self.waypoints = None
        self.obs_inner_waypoints= None
        self.obs_outer_waypoints = None
        self.standard_wapoints=None
        self.colors = None
        self.obstacle_detected = False  # 장애물 탐지 여부
        self.base_lookahead_distance = 1.5  # 기본 lookahead 거리
        self.min_lookahead_distance = 0  # 최소 lookahead 거리
        # self.max_lookahead_distance = 3.0  # 최대 lookahead 거리
        self.speed_factor = 0.7  # 속도에 따른 조정 인자
        self.wheelbase_length = 0.3302  # 차량 휠베이스 길이 (미터)
        self.current_pose = None
        self.current_speed = 0
        self.angle_threshold = np.pi/3  # 전방 웨이포인트 허용 각도 범위 (45도)
        self.yaw = 0.0  # 차량의 현재 방위각
        self.target_point=None 
        self.lane_change_triggered = False
        self.reduced_lookahead_factor = 0.5
        self.is_in_overtaking_path = False  # 오버테이킹 경로에 있는지 여부
        self.reduced_lookahead_duration = 10

    # 웨이포인트 콜백

    def waypoints_callback(self, msg):
        self.waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    # 장애물 웨이포인트 콜백
    def obs_inner_waypoints_callback(self, msg):
        self.obs_inner_waypoints = np.array([[pose.x, pose.y] for pose in msg.points])

    def obs_outer_waypoints_callback(self, msg):
        self.obs_outer_waypoints = np.array([[pose.x, pose.y] for pose in msg.points])

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

    def obstacle_position_callback(self, msg):
        # 장애물 위치 업데이트
        self.obstacle_position = (msg.point.x, msg.point.y)

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

    def is_lane_change_required(self):
        # 현재 오버테이킹 경로에 있고 장애물이 감지되었을 때 차선 변경 필요
        if (
            self.is_on_overtaking_path()
            and self.obstacle_detected
            and not self.is_in_overtaking_path
        ):
            self.is_in_overtaking_path = True  # 차선 변경을 시작했음을 나타냄
            return True
        # 오버테이킹 경로에서 벗어나고 장애물이 사라지면 원래 차선으로 돌아옴
        elif self.is_in_overtaking_path and not self.obstacle_detected:
            self.is_in_overtaking_path = False  # 원래 차선으로 돌아감
            return True
        return False

    def is_obstacle_on_right(self):
        # 장애물의 x, y 좌표를 기반으로 오른쪽 또는 왼쪽에 있는지 확인
        if self.obstacle_position is None or self.current_pose is None:
            return False  # 장애물 위치 정보가 없는 경우 기본값 False 반환

        obs_x, obs_y = self.obstacle_position
        current_x, current_y = (
            self.current_pose.position.x,
            self.current_pose.position.y,
        )
        angle = np.arctan2(obs_y - current_y, obs_x - current_x) - self.yaw
        return np.sin(angle) > 0  # 오른쪽이면 True 반환

    def reset_lane_change_flag(self):
        # 차선 변경 완료 후 플래그를 초기화하여 다시 줄일 수 있도록 준비
        if not self.is_lane_change_required():
            self.lane_change_triggered = False

    def compute_lookahead_distance(self, target_color):
        # target_color에 따라 기본 룩어헤드 거리 계산
        if target_color is None:
            lookahead_distance = self.base_lookahead_distance
        else:
            red = target_color[0]
            green = target_color[1]
            lookahead_distance = self.base_lookahead_distance + 0.5 * green - 1 * red

        # 차선 변경이 처음 필요할 때만 줄어든 lookahead 적용 및 타이머 시작
        if self.is_lane_change_required() and not self.lane_change_triggered:
            lookahead_distance *= self.reduced_lookahead_factor  # 줄어든 거리 적용
            self.lane_change_triggered = True  # 플래그 설정
            rospy.Timer(
                rospy.Duration(self.reduced_lookahead_duration),
                self.reset_lane_change_trigger,
                oneshot=False,
            )

        # 최소 룩어헤드 거리로 제한 및 None 값 방지
        lookahead_distance = max(self.min_lookahead_distance, lookahead_distance)
        return lookahead_distance

    def reset_lookahead_distance(self, event):
        # 일정 시간이 지나면 룩어헤드 거리를 기본 값으로 복원
        self.lane_change_triggered = False

    def reset_lane_change_trigger(self, event):
        # 0.8초 후 lane_change_triggered 플래그 해제
        self.lane_change_triggered = False

    def get_closest_waypoint(self):
        """
        Calculate the index of the closest waypoint to the current vehicle position.
        Returns:
            closest_idx (int): closest waypoint index
            closest_dist (float): distance to the closest waypoint
        """
        if self.waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float('inf')
        closest_idx = None

        # 차량의 현재 위치 가져오기
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # 각 웨이포인트에 대해 거리를 계산
        for i, waypoint in enumerate(self.waypoints):
            dist = np.linalg.norm(np.array([waypoint[0] - current_x, waypoint[1] - current_y]))
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        return closest_idx, closest_dist
    # 타겟 포인트 선택 함수 (장애물 유무에 따라 경로를 선택)
    def get_target_point(self, lookahead_distance):
        if self.obstacle_detected and self.is_on_overtaking_path():
            # 장애물이 있는 위치를 기준으로 경로 선택
            if self.is_obstacle_on_right():
                current_waypoints = self.obs_inner_waypoints
            else:
                current_waypoints = self.obs_outer_waypoints
        else:
            current_waypoints = self.waypoints

        if current_waypoints is None or self.current_pose is None:
            return None, None

        closest_dist, target_point, target_color = float("inf"), None, None
        best_match_diff = float("inf")
        for idx, point in enumerate(current_waypoints):
            dist = np.linalg.norm(
                [
                    point[0] - self.current_pose.position.x,
                    point[1] - self.current_pose.position.y,
                ]
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
                        if idx < len(self.colors):
                            target_color = self.colors[idx]
        if target_point is not None:
            self.publish_marker(target_point)
        return target_point, target_color

    def is_obstacle_on_right(self):
        if not self.obstacle_points or self.current_pose is None:
            return False  # 기본값

        right_count, left_count = 0, 0
        current_x, current_y = (
            self.current_pose.position.x,
            self.current_pose.position.y,
        )

        # 각 장애물 포인트의 상대 위치를 확인
        for obs_x, obs_y in self.obstacle_points:
            # 장애물 위치에서 차량 위치로의 벡터
            vector_x, vector_y = obs_x - current_x, obs_y - current_y
            # 차량의 진행 방향과 장애물 벡터의 방향 각도 차이 계산
            angle = np.arctan2(vector_y, vector_x) - self.yaw
            angle = np.arctan2(np.sin(angle), np.cos(angle))  # -π ~ π 사이로 정규화

            if angle < 0:
                right_count += 1
            else:
                left_count += 1

        # 다수결 방식으로 방향 판단 (오른쪽에 더 많은 경우 True 반환)
        return right_count > left_count

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

        return steering_angle

    # 속도 계산 함수 (색상을 기반으로 기본 속도 계산, 장애물 여부에 따라 조정)
    def compute_speed(self, target_color):
        if target_color is None:
            speed = 2
        else:
            red = target_color[0]
            green = target_color[1]
            speed = 2 + 2 * green - 1.0 * red

        if self.obstacle_detected:
            if self.is_on_overtaking_path():
                return speed  # 장애물이 있지만 오버테이킹 경로에 있을 경우 기본 속도 유지
            else:
                return max(
                    0 * 0.5, 0.1
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

                # 차선 변경이 완료되면 플래그 초기화
                self.reset_lane_change_flag()

                # 그 후 target_point를 선택
                self.target_point, _ = self.get_target_point(lookahead_distance)

                # 조향각 계산
                steering_angle = self.compute_steering_angle(
                    self.target_point, lookahead_distance
                )

                # 속도 및 조향을 포함한 Ackermann 메시지 생성
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = steering_angle
                drive_msg.drive.speed = self.compute_speed(target_color)

                # 메시지를 퍼블리시
                self.drive_pub.publish(drive_msg)
                lookahead=Float64()
                lookahead.data=lookahead_distance
                self.lookahead_pub.publish(lookahead)

                # 디버그용으로 현재 타겟 포인트와 룩어헤드 거리를 출력
                # rospy.loginfo(f"Current target point: {self.target_point}")
                # rospy.loginfo(f"Lookahead distance: {lookahead_distance}")
                # rospy.logwarn("No target point available, skipping control update.")
            else:
                pass
                # rospy.logwarn("No closest waypoint found, skipping control update.")
            # print(self.is_on_overtaking_path())
            print(self.is_obstacle_on_right())
            # 루프 딜레이 설정
            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
