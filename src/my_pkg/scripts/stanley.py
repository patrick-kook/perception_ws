import rospy
import numpy as np
from geometry_msgs.msg import (
    PoseArray,
    PoseWithCovarianceStamped,
    Vector3Stamped,
    PointStamped,
)
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Joy
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import time

class PurePursuit:

    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publisher for Ackermann drive messages
        # self.steer_pub = rospy.Publisher(
        #     "/commands/servo/position", Float64, queue_size=10
        # )
        # self.speed_pub = rospy.Publisher(
        #     "/commands/motor/duty_cycle", Float64, queue_size=10
        # )
        self.drive_pub=rospy.Publisher("/drive",AckermannDriveStamped,queue_size=10)

        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)

        # ì›¨ì´í¬ì¸íŠ¸ì™€ ì˜¤ë„ë©”íŠ¸ë¦¬ êµ¬ë…ìž
        rospy.Subscriber(
            "/trajectory", Marker, self.waypoints_callback
        )
        # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber("/odom",Odometry, self.odom_callback)
        # ìž¥ì• ë¬¼ ê°ì§€ ë° ì†ë„ êµ¬ë…ìž
        rospy.Subscriber("/obstacle_detected", Bool, self.obstacle_callback)
        rospy.Subscriber("/detected_obstacles", PointStamped, self.diff_callback)

        rospy.Subscriber(
            "/detected_obstacle_velocity",
            Vector3Stamped,
            self.obstacle_velocity_callback,
        )

        # Joy callback
        rospy.Subscriber("/joy", Joy, self.joy_CB)
        self.joy_msg = Joy()
        self.joy_mode = False  # false : stop, true : auto drive
        self.steer = 0.5
        self.speed = 0
        self.diff_msg = PointStamped()
        # PID ì œì–´ ë³€ìˆ˜ ì´ˆê¸°í™”
        self.prev_error = 0.0
        self.integral = 0.0
        self.Kp = 0.8  # ë¹„ë¡€ ê²Œì¸
        self.Ki = 0.01  # ì ë¶„ ê²Œì¸
        self.Kd = 0.1  # ë¯¸ë¶„ ê²Œì¸

        self.tflag = False
        # ë§¤ê°œë³€ìˆ˜ ì´ˆê¸°í™”
        self.prev_steering_angle = 0.0  # ë§ˆì§€ë§‰ ìŠ¤í‹°ì–´ë§ ê°’ì„ ì´ˆê¸°í™”
        self.obstacle_detected = False  # ìž¥ì• ë¬¼ ê°ì§€ ì—¬ë¶€
        self.obstacle_velocity = 0.0  # ìž¥ì• ë¬¼ ì†ë„
        self.waypoints = None
        self.colors = None
        self.base_lookahead_distance = 1  # ê¸°ë³¸ lookahead ê±°ë¦¬
        self.min_lookahead_distance = 0  # ìµœì†Œ lookahead ê±°ë¦¬
        self.wheelbase_length = 0.3302  # ì°¨ëŸ‰ íœ ë² ì´ìŠ¤ ê¸¸ì´ (ë¯¸í„°)
        self.current_pose = None
        self.current_speed = 0
        self.angle_threshold = (
            np.pi / 2.5
        )  # ì „ë°© ì›¨ì´í¬ì¸íŠ¸ í—ˆìš© ê°ë„ ë²”ìœ„ (45ë„)
        self.yaw = 0.0  # ì°¨ëŸ‰ì˜ í˜„ìž¬ ë°©ìœ„ê°
        self.target_point = None
        self.previous_pose = None
        self.previous_time = None

    # ì›¨ì´í¬ì¸íŠ¸ ì½œë°±
    def waypoints_callback(self, msg):
        self.waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    def joy_CB(self, msg):
        self.joy_msg = msg
        self.speed = 0.15 * self.joy_msg.axes[1]
        self.steer = (self.joy_msg.axes[3]) / 2 + 0.5

        if self.joy_msg.buttons[0] == 1:
            self.joy_mode = True
        if self.joy_msg.buttons[1] == 1:
            self.joy_mode = False

    def diff_callback(self, msg):
        self.diff_msg = msg

        # print("x = ", self.current_pose.position.x,self.diff_msg.point.x)
        # print("y = ", self.current_pose.position.y,self.diff_msg.point.y)

        my_x = self.current_pose.position.x
        my_y = self.current_pose.position.y
        op_x = self.diff_msg.point.x
        op_y = self.diff_msg.point.y

        p1 = np.array([my_x, my_y])
        p2 = np.array([op_x, op_y])
        self.dist = np.linalg.norm(p1 - p2)
        # print(dist)

    # ì˜¤ë„ë©”íŠ¸ë¦¬ ì½œë°±
    def odom_callback(self, msg):
        if self.tflag == False:
            rospy.loginfo("sub_time")
            self.tflag = True
        self.current_pose = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(orientation_list)

    # ìž¥ì• ë¬¼ ê°ì§€ ì½œë°±
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data  # ìž¥ì• ë¬¼ ê°ì§€ ì—¬ë¶€ ì—…ë°ì´íŠ¸

    # ìž¥ì• ë¬¼ ì†ë„ ì½œë°±
    def obstacle_velocity_callback(self, msg):
        self.obstacle_velocity = np.hypot(
            msg.vector.x, msg.vector.y
        )  # ìž¥ì• ë¬¼ ì†ë„ ê³„ì‚°

    # ë£©ì–´í—¤ë“œ ê±°ë¦¬ ê³„ì‚°
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
        lookahead_distance = self.base_lookahead_distance 

        # Ensure the lookahead distance is within the specified range
        lookahead_distance = max(self.min_lookahead_distance, lookahead_distance)

        return lookahead_distance

    # ê°€ìž¥ ê°€ê¹Œìš´ ì›¨ì´í¬ì¸íŠ¸ ì¸ë±ìŠ¤ ì°¾ê¸°
    def get_closest_waypoint(self):
        if self.waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        closest_idx = None
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        for i, waypoint in enumerate(self.waypoints):
            dist = np.linalg.norm(
                np.array([waypoint[0] - current_x, waypoint[1] - current_y])
            )
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        return closest_idx, closest_dist

    def angle_threshold_func(self):
        target_color = None
        closest_idx, closest_dist = self.get_closest_waypoint()
        if closest_idx is not None and closest_dist is not None:
            target_color = (
                self.colors[closest_idx] if closest_idx < len(self.colors) else None
            )
            _, green, _, _ = target_color
            self.angle_threshold -= 1.0 * green

    # íƒ€ê²Ÿ í¬ì¸íŠ¸ ì„ íƒ
    def get_target_point(self, lookahead_distance):
        current_waypoints = self.waypoints
        if current_waypoints is None or self.current_pose is None:
            return None, None

        closest_dist = float("inf")
        target_point = None
        target_color = None
        best_match_diff = float("inf")

        # ì›¨ì´í¬ì¸íŠ¸ë“¤ ì‚¬ì´ì—ì„œ lookahead ê±°ë¦¬ ì´í•˜ ì¤‘ ê°€ìž¥ ê°€ê¹Œìš´ í¬ì¸íŠ¸ë¥¼ ì°¾ìŒ
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

                        # ìƒ‰ìƒ ì¸ë±ìŠ¤ê°€ ë²”ìœ„ë¥¼ ë²—ì–´ë‚˜ì§€ ì•Šë„ë¡ ì²´í¬
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

    def compute_vehicle_speed(self, current_pose):
        """
        차량 속도를 계산하는 함수
        :param current_pose: 현재 위치 (x, y, theta)
        :return: 차량 속도 (m/s)
        """  
        current_time = time.time()

        # 이전 위치가 없으면 초기화
        if self.previous_pose is None or self.previous_time is None:
            self.previous_pose = current_pose
            self.previous_time = current_time
            return 0.0  # 초기 속도는 0으로 설정

        # 이전 위치와 현재 위치 간의 거리 계산
        dx = current_pose.position.x - self.previous_pose.position.x
        dy = current_pose.position.y - self.previous_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # 시간 차이 계산
        time_delta = current_time - self.previous_time
        if time_delta <= 0:
            return 0.0  # 시간 차이가 0 이하이면 속도 계산 불가

        # 속도 계산
        speed = distance / time_delta

        # 이전 위치 및 시간 업데이트
        self.previous_pose = current_pose
        self.previous_time = current_time

        return speed

    # ì¡°í–¥ê° ê³„ì‚°

    def compute_steering_angle(self, target_point, waypoints, speed, k=10):
        """
        스탠리 알고리즘으로 조향 각도를 계산하는 클래스 메서드
        :param target_point: 목표 지점 (x, y)
        :param waypoints: 경로 점 리스트 [(x1, y1), (x2, y2), ...]
        :param speed: 차량 속도 (m/s)
        :param k: 스탠리 게인 (기본값: 1.0)
        :return: 조향 각도 (radians)
        """
        if target_point is None:
            return 0.0  # 목표 지점이 없으면 조향 각도는 0

        # 현재 위치와 헤딩 정보 가져오기
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.yaw  # 차량의 현재 yaw

        # 가장 가까운 경로 점 찾기 (크로스 트랙 에러 계산에 사용)
        closest_distance = float("inf")
        closest_point = None
        for waypoint in waypoints:
            dx = waypoint[0] - current_x
            dy = waypoint[1] - current_y
            distance = np.sqrt(dx**2 + dy**2)
            if distance < closest_distance:
                closest_distance = distance
                closest_point = waypoint

        if closest_point is None:
            return 0.0  # 경로 점이 없으면 조향 각도는 0

        # 크로스 트랙 에러 (CTE)
        dx = closest_point[0] - current_x
        dy = closest_point[1] - current_y
        cross_track_error = np.hypot(dx, dy)  # 유클리드 거리

        # 경로와 차량의 헤딩 차이
        path_yaw = np.arctan2(dy, dx)  # 경로 방향
        heading_error = path_yaw - current_yaw
        heading_error = np.arctan2(
            np.sin(heading_error), np.cos(heading_error)
        )  # -pi ~ pi로 정규화

        # 스탠리 알고리즘 계산
        cross_track_steering = np.arctan(
            k * cross_track_error / (speed + 1e-5)
        )  # 속도에 따라 크기 조정
        steering_angle = heading_error + cross_track_steering

        return steering_angle

    # ì†ë„ ê³„ì‚°
    def compute_speed(self, target_color):
        if self.obstacle_detected:
            # ìž¥ì• ë¬¼ ì†ë„ë¥¼ ë”°ë¼ê°€ëŠ” PID ì œì–´
            error = self.dist * 0.06
            # self.integral += error
            # derivative = error - self.prev_error
            # self.prev_error = error
            # print(self.dist)

            # # PID ì†ë„ ì œì–´ ì ìš©
            pid_speed = (
                2 * self.Kp * error
            )  #  self.Ki * self.integral + self.Kd * derivative
            # print(pid_speed ," ",self.dist)
            return max(0.04, pid_speed)  # ì†ë„ëŠ” ìŒìˆ˜ê°€ ë˜ì§€ ì•Šë„ë¡ ì œí•œ
            # return 0.06
        else:
            # ìž¥ì• ë¬¼ì´ ì—†ëŠ” ê²½ìš° ê¸°ë³¸ ì†ë„ ê³„ì‚°
            if target_color is None:
                return 2
            else:
                red, green, _, _ = target_color
                return 1 + 0.1 * green - 0.1 * red

    # ì‹¤í–‰ í•¨ìˆ˜


    def run(self):
        while not rospy.is_shutdown():
            self.angle_threshold = np.pi / 2.5
            self.angle_threshold_func()

            # 기본값 설정
            steering_angle = 0.0
            speed = 0.0
            target_color = None  # target_color 초기화

            # joy_mode가 False일 때는 수동 조작을 하지 않음
            if not self.joy_mode:
                steer_msg = Float64()
                speed_msg = Float64()
                steer_msg.data = self.steer
                speed_msg.data = self.speed

            else:
                # 가장 가까운 경로 점 찾기
                closest_idx, closest_dist = self.get_closest_waypoint()

                if closest_idx is not None and closest_dist is not None:
                    target_color = (
                        self.colors[closest_idx] if closest_idx < len(self.colors) else None
                    )

                    # lookahead_distance 계산
                    lookahead_distance = self.compute_lookahead_distance(target_color)

                    # 목표 지점 계산
                    self.target_point, _ = self.get_target_point(lookahead_distance)

                    if self.target_point is not None:
                        # 차량 속도 계산
                        speed = self.compute_vehicle_speed(self.current_pose)

                        # 스탠리 알고리즘을 사용하여 조향 각도 계산
                        steering_angle = self.compute_steering_angle(
                            self.target_point, self.waypoints, speed
                        )

                        self.prev_steering_angle = steering_angle
                    else:
                        steering_angle = self.prev_steering_angle

            # Drive message 준비
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = self.compute_speed(target_color)

            # 차량 제어 메시지 전송
            self.drive_pub.publish(drive_msg)

            # 루프 주기마다 sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
