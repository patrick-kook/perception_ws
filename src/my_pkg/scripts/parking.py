import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Vector3Stamped, PointStamped
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Joy
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import time

class PurePursuit:

    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publisher for Ackermann drive messages
        self.steer_pub = rospy.Publisher(
            "/commands/servo/position", Float64, queue_size=10
        )
        self.speed_pub = rospy.Publisher(
            "/commands/motor/duty_cycle", Float64, queue_size=10
        )
        #self.drive_pub=rospy.Publisher("/drive",AckermannDriveStamped,queue_size=10)

        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)
        self.parking_step = 0
        # 웨이포인트와 오도메트리 구독자
        rospy.Subscriber(
            "/trajectory", Marker, self.waypoints_callback
        )
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        #rospy.Subscriber("/odom",Odometry, self.odom_callback)
        # 장애물 감지 및 속도 구독자
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
        self.joy_mode = False # false : stop, true : auto drive
        self.steer = 0.6
        self.speed = 0
        self.diff_msg = PointStamped()
        # PID 제어 변수 초기화
        self.prev_error = 0.0
        self.integral = 0.0
        self.Kp = 0.8  # 비례 게인
        self.Ki = 0.01  # 적분 게인
        self.Kd = 0.1  # 미분 게인

        self.tflag = False
        # 매개변수 초기화
        self.prev_steering_angle = 0.0  # 마지막 스티어링 값을 초기화
        self.obstacle_detected = False  # 장애물 감지 여부
        self.obstacle_velocity = 0.0  # 장애물 속도
        self.waypoints = None
        self.colors = None 
        self.base_lookahead_distance = 0.5 # 기본 lookahead 거리
        self.min_lookahead_distance = 0  # 최소 lookahead 거리
        self.wheelbase_length = 0.2702  # 차량 휠베이스 길이 (미터)
        self.current_pose = None
        self.current_speed = 0
        self.angle_threshold = np.pi*1.5  # 전방 웨이포인트 허용 각도 범위 (45도)
        self.yaw = 0.0  # 차량의 현재 방위각
        self.target_point=None
        self.last_pause_time = None
        self.target_color = None

    # 웨이포인트 콜백
    def waypoints_callback(self, msg):
        self.waypoints = np.array([[pose.x, pose.y] for pose in msg.points])
        self.colors = np.array(
            [[color.r, color.g, color.b, color.a] for color in msg.colors]
        )

    def joy_CB(self,msg):
        self.joy_msg = msg
        self.speed = 5 * self.joy_msg.axes[1]
        self.steer = (self.joy_msg.axes[3])/2 

        if self.joy_msg.buttons[0] == 1 :
            self.joy_mode = True
        if self.joy_msg.buttons[1] == 1 :
            self.joy_mode = False

    def diff_callback(self,msg):
        self.diff_msg = msg

        # print("x = ", self.current_pose.position.x,self.diff_msg.point.x)
        # print("y = ", self.current_pose.position.y,self.diff_msg.point.y)

        my_x = self.current_pose.position.x
        my_y = self.current_pose.position.y
        op_x = self.diff_msg.point.x
        op_y = self.diff_msg.point.y

        p1 = np.array([my_x,my_y])
        p2 = np.array([op_x,op_y])
        self.dist = np.linalg.norm(p1-p2)
        # print(dist)

    # 오도메트리 콜백
    def odom_callback(self, msg):
        if(self.tflag == False):
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

    # 장애물 감지 콜백
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data  # 장애물 감지 여부 업데이트

    # 장애물 속도 콜백
    def obstacle_velocity_callback(self, msg):
        self.obstacle_velocity = np.hypot(
            msg.vector.x, msg.vector.y
        )  # 장애물 속도 계산

    # 룩어헤드 거리 계산
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
        lookahead_distance = self.base_lookahead_distance + 0.1 * green - 0 * red

        # Ensure the lookahead distance is within the specified range
        lookahead_distance = max(self.min_lookahead_distance, lookahead_distance)

        return lookahead_distance

    # 가장 가까운 웨이포인트 인덱스 찾기
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
        target_color=None
        closest_idx, closest_dist = self.get_closest_waypoint()
        if closest_idx is not None and closest_dist is not None:
            target_color = (
                self.colors[closest_idx] if closest_idx < len(self.colors) else None
            )
            _, green, _, _ = target_color
            self.angle_threshold-= 1.0 * green 

    # 타겟 포인트 선택
    def get_target_point(self, lookahead_distance):
        current_waypoints = ( self.waypoints
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
                        np.arctan2(np.sin(heading - self.yaw), np.cos(heading - self.yaw))
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

    # 조향각 계산
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

    def check_and_pause(self, target_color):
        """
        빨간색(Red)이 감지되었을 때 딱 한 번만 5초 멈춥니다.
        이후 1분 동안은 재실행되지 않습니다.

        Args:
            target_color (list): 타겟 웨이포인트의 색상 정보 [r, g, b, a].
        """
        current_time = time.time()

        # 1분 쿨다운 확인
        if self.last_pause_time is not None and current_time - self.last_pause_time < 10:
            rospy.loginfo("Pause function is in cooldown. Skipping.")
            return

        # 빨간색 감지 여부 확인
        if target_color is not None:
            red, green, _, _ = target_color
            if red > 0.5 and green < 0.5:  # 빨간색 감지 기준
                rospy.loginfo("Red detected! Pausing for 5 seconds.")

                # 5초 동안 멈춤
                start_time = time.time()
                while time.time() - start_time < 5:
                    # speed_msg = Float64()
                    # steer_msg = Float64()
                    # steer_msg.data=0.0
                    # speed_msg.data=0.0
                    # self.speed_pub.publish(speed_msg)
                    # self.steer_pub.publish(steer_msg)

                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.steering_angle = 0.0  # 정지 시 조향각 0 유지
                    drive_msg.drive.speed = 0.0  # 속도 0으로 설정
                    self.drive_pub.publish(drive_msg)
                    self.rate.sleep()

                rospy.loginfo("Resuming operation.")
                self.last_pause_time = current_time  # 마지막 실행 시간 업데이트

    # 속도 계산
    def compute_speed(self, target_color):
        current_time = rospy.Time.now().to_sec()
        if self.obstacle_detected:
            # 장애물 속도를 따라가는 PID 제어
            error = self.dist*0.06
            # self.integral += error
            # derivative = error - self.prev_error
            # self.prev_error = error
            # print(self.dist)

            # # PID 속도 제어 적용
            pid_speed = 2 * self.Kp * error #  self.Ki * self.integral + self.Kd * derivative
            # print(pid_speed ," ",self.dist)
            return max(0.0, pid_speed)  # 속도는 음수가 되지 않도록 제한
            # return 0.06
        else:
            if target_color is None:
                return 0.06

            # target_color가 제공된 경우
            else:
                red, green, _, _ = target_color

                # 색상 정보에 따라 속도 계산
                computed_speed = 0.4 +  0.3 * green - 0.4 * red

                return computed_speed  # 계산된 속도 반환
    
    def check_blue_waypoint(self):
        if self.colors is None or self.waypoints is None:
            return  # 웨이포인트 정보가 없는 경우 종료
        closest_idx, _ = self.get_closest_waypoint()
        if closest_idx is not None and closest_idx < len(self.colors):
            target_color = self.colors[closest_idx]
            blue_channel = target_color[2]  # B (blue) 채널 값
            if blue_channel > 0.8:  # 파란색으로 인식하는 임계값 (0.8 이상)
                self.parking_step = 1
    
    # 실행 함수
    def run(self):
        while not rospy.is_shutdown():
            self.angle_threshold = np.pi / 2.5
            
            if self.parking_step == 1:
            # Parking algorithm
                if not hasattr(self, 'start_time') or self.start_time is None:
                    self.start_time = rospy.Time.now()
                    self.parking_phase = 0  # Initialize parking phase

                    elapsed_time = (rospy.Time.now() - self.start_time).to_sec()

                    steer_msg = Float64()
                    speed_msg = Float64()

            # Phase 1: 3 seconds of straight driving
            if self.parking_phase == 0:
                if elapsed_time < 3.0:
                    steer_msg.data = 0.0  # Straight steering
                    speed_msg.data = 0.15  # Forward speed
                else:
                    self.parking_phase = 1
                    self.start_time = rospy.Time.now()  # Reset timer

            # Phase 2: 5 seconds of stopping
            elif self.parking_phase == 1:
                if elapsed_time < 5.0:
                    steer_msg.data = 0.0
                    speed_msg.data = 0.0  # Stop
                else:
                    self.parking_phase = 2
                    self.start_time = rospy.Time.now()  # Reset timer

            # Phase 3: 3 seconds of reverse driving
            elif self.parking_phase == 2:
                if elapsed_time < 3.0:
                    steer_msg.data = 0.0  # Straight steering
                    speed_msg.data = -0.15  # Reverse speed
                else:
                    self.parking_phase = 3
                    self.start_time = rospy.Time.now()  # Reset timer

            # Phase 4: 3 seconds of forward driving
            elif self.parking_phase == 3:
                if elapsed_time < 3.0:
                    steer_msg.data = 0.0  # Straight steering
                    speed_msg.data = 0.15  # Forward speed
                else:
                    self.parking_phase = 4  # End parking process

            # Publish messages
            self.steer_pub.publish(steer_msg)
            self.speed_pub.publish(speed_msg)

            # Stop the algorithm if parking is complete
            if self.parking_phase == 4:
                rospy.loginfo("Parking complete. Stopping robot.")
                break

            elif self.joy_mode == False:
                steer_msg = Float64()
                speed_msg = Float64()
                steer_msg.data = self.steer
                speed_msg.data = self.speed

            else :
                closest_idx, closest_dist = self.get_closest_waypoint()
                if closest_idx is not None and closest_dist is not None:
                    target_color = (
                        self.colors[closest_idx] if closest_idx < len(self.colors) else None
                    )
                    lookahead_distance = self.compute_lookahead_distance(target_color)
                    self.target_point, _ = self.get_target_point(lookahead_distance)

                    if self.target_point is not None:
                        steering_angle = self.compute_steering_angle(
                            self.target_point, lookahead_distance
                        )
                        self.prev_steering_angle = steering_angle
                    else:
                        steering_angle = self.prev_steering_angle
                    steer_msg = Float64()
                    speed_msg = Float64()


                    #self.speed = 0.05 * self.joy_msg.axes[1]
                    #self.steer = (self.joy_msg.axes[2])/2 + 0.5
            

                    steer_msg.data = steering_angle
                    speed_msg.data = self.compute_speed(target_color)
                    
                
            #rospy.loginfo("pub_time")
            self.steer_pub.publish(steer_msg)
            self.speed_pub.publish(speed_msg)
            #print(speed_msg)
                # drive_msg=AckermannDriveStamped()
                # drive_msg.drive.steering_angle=steering_angle
                # drive_msg.drive.speed=self.compute_speed(target_color)

                # self.drive_pub.publish(drive_msg)
            self.rate.sleep()



if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass