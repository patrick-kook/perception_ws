import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import tf
import casadi as ca


class MPCController:
    def __init__(self):
        rospy.init_node("mpc_controller", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS 퍼블리셔
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # ROS 서브스크라이버
        rospy.Subscriber(
            "/global_path/optimal_trajectory", Marker, self.trajectory_callback
        )
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 차량 파라미터
        self.dt = 0.2  # 시간 간격 (초)
        self.N = 15  # 예측 지평선
        self.wheelbase_length = 0.3302  # 휠베이스 길이 (미터)
        self.max_steer = np.radians(30)  # 최대 조향 각도 (라디안)
        self.max_speed = 7 # 최대 속도 (미터/초)
        self.min_speed = 3 

        # 상태 변수
        self.current_pose = None
        self.yaw = 0.0
        self.trajectory = None
        self.current_speed = 0.0

    def trajectory_callback(self, msg):
        """웨이포인트를 저장 및 색상 정보를 사용하여 가속/감속 결정"""
        self.trajectory = []
        self.colors = []
        for i, p in enumerate(msg.points):
            self.trajectory.append([p.x, p.y])
            self.colors.append([msg.colors[i].r, msg.colors[i].g, msg.colors[i].b])
        self.trajectory = np.array(self.trajectory)

    def odom_callback(self, msg):
        """현재 차량의 위치와 자세를 업데이트"""
        self.current_pose = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(orientation_list)

    def get_horizon_waypoints(self, waypoints, colors, x0):
        """차량의 현재 위치를 기준으로 Horizon 웨이포인트를 동적으로 추출"""
        current_position = x0[:2]

        # 웨이포인트와의 거리 계산
        distances = np.linalg.norm(waypoints - current_position, axis=1)

        # 가장 가까운 웨이포인트 찾기
        closest_idx = np.argmin(distances)

        # Horizon만큼의 웨이포인트 반환
        horizon_waypoints = waypoints[closest_idx : closest_idx + self.N + 1]
        horizon_colors = colors[closest_idx]
        return horizon_waypoints, horizon_colors


    def solve_mpc(self, x0, waypoints, colors):
        """
        Solve the MPC optimization problem using CasADi with dynamic Q and R values based on color influence.
        """
        import casadi as ca
        import numpy as np

        # Define optimization variables
        x = ca.MX.sym("x", 4, self.N + 1)  # States: [x, y, yaw, speed]
        u = ca.MX.sym("u", 2, self.N)  # Controls: [steering_angle, acceleration]

        # Define parameters (initial state, waypoints)
        P = ca.MX.sym("P", 4 + 2 * self.N)  # Initial state + waypoints

        # Objective and constraints
        obj = 0
        g = []

        # Dynamically adjust Q and R based on colors
        Q_xy_cost = np.clip(colors[0], 0.2, 1)  # Q for x, y (0.2 to 1)
        Q_theta_cost = 1.0  # Fixed for theta
        Q_speed_cost = 5.0  # Fixed for speed

        R_steering_cost = np.clip(colors[1] * 4.0, 0.1, 10)  # R for steering (0.1 to 10)
        R_acceleration_cost = 0.5  # Fixed for acceleration

        Q = np.diag([Q_xy_cost, Q_xy_cost, Q_theta_cost, Q_speed_cost])  # State cost
        R = np.diag([R_steering_cost, R_acceleration_cost])  # Control cost

        # State initialization constraint
        g.append(x[:, 0] - P[0:4])  # x0 constraint

        for k in range(self.N):
            # Current waypoint
            target = P[4 + 2 * k : 4 + 2 * k + 2]

            # Dynamics model
            x_next = ca.vertcat(
                x[0, k] + self.dt * x[3, k] * ca.cos(x[2, k]),
                x[1, k] + self.dt * x[3, k] * ca.sin(x[2, k]),
                x[2, k] + self.dt * x[3, k] * ca.tan(u[0, k]) / self.wheelbase_length,
                x[3, k] + self.dt * u[1, k],
            )

            g.append(x[:, k + 1] - x_next)  # Dynamics constraint

            # Objective (tracking error + control effort)
            state_error = x[0:2, k] - target  # Tracking error for x, y
            obj += ca.mtimes([state_error.T, Q[0:2, 0:2], state_error])  # State cost
            obj += Q[3, 3] * (x[3, k] - np.clip(3 + 4 * colors[1], 3, 7)) ** 2  # Speed cost
            obj += ca.mtimes([u[:, k].T, R, u[:, k]])  # Control effort

        # Constraints on inputs
        g = ca.vertcat(*g)

        # Create optimization problem
        opt_vars = ca.vertcat(ca.reshape(x, -1, 1), ca.reshape(u, -1, 1))
        nlp = {"x": opt_vars, "f": obj, "g": g, "p": P}

        # Solver settings
        opts = {"ipopt.print_level": 0, "print_time": 0}
        solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        # Ensure waypoints have the correct dimension
        waypoints_flat = waypoints.flatten()
        if len(waypoints_flat) > 2 * self.N:
            waypoints_flat = waypoints_flat[: 2 * self.N]  # Truncate if too many waypoints
        elif len(waypoints_flat) < 2 * self.N:
            waypoints_flat = np.pad(
                waypoints_flat, (0, 2 * self.N - len(waypoints_flat))
            )  # Pad if too few waypoints

        # Combine initial state and waypoints
        p = np.concatenate([x0, waypoints_flat])

        # Solve the optimization problem
        sol = solver(
            x0=np.zeros(opt_vars.shape[0]), lbx=-np.inf, ubx=np.inf, lbg=0, ubg=0, p=p
        )
        sol_u = ca.reshape(sol["x"][4 * (self.N + 1) :], 2, self.N)

        return sol_u[:, 0]  # First control input

    def run(self):
        while not rospy.is_shutdown():
            if self.current_pose is None or self.trajectory is None:
                rospy.loginfo("현재 위치 또는 웨이포인트를 기다리는 중입니다...")
                self.rate.sleep()
                continue

            # 현재 상태 정의: [x, y, yaw, speed]
            x0 = np.array(
                [
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.yaw,
                    self.current_speed,
                ]
            )

            # 현재 위치를 기준으로 Horizon 웨이포인트 추출
            waypoints, colors = self.get_horizon_waypoints(self.trajectory, self.colors, x0)
            if len(waypoints) < self.N + 1:
                rospy.logwarn("Horizon 웨이포인트가 부족합니다.")
                self.rate.sleep()
                continue

            # MPC 최적화 문제 풀이
            control_input = self.solve_mpc(x0, waypoints, colors)
            if control_input is None:
                self.rate.sleep()
                continue

            # 제어 입력 추출
            steering_angle = float(control_input[0])
            acceleration = float(control_input[1])

            # 속도 업데이트 및 최대 속도 제한
            self.current_speed += acceleration * self.dt
            if self.current_speed > self.max_speed:
                self.current_speed = self.max_speed
            elif self.current_speed < self.min_speed:
                self.current_speed = self.min_speed

            # 컬러 값을 기반으로 속도 제어
            r = colors[0]
            g = colors[1]

            # r과 g 값을 기반으로 목표 속도 계산
            target_speed = self.min_speed + (self.max_speed - self.min_speed) * (g - r)

            # 목표 속도에 도달하도록 현재 속도 조정
            if self.current_speed < target_speed:
                self.current_speed += 0.1  # 가속
            elif self.current_speed > target_speed:
                self.current_speed -= 0.1  # 감속

            # ROS 메시지로 제어 입력 발행
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = self.current_speed

            self.drive_pub.publish(drive_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        mpc = MPCController()
        mpc.run()
    except rospy.ROSInterruptException:
        pass
