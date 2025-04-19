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

        # ROS Publishers
        self.drive_pub = rospy.Publisher(
            "/drive", AckermannDriveStamped, queue_size = 10
        )
        # self.steer_pub = rospy.Publisher(
        #     "/commands/servo/position", Float64, queue_size=10
        # )
        # self.speed_pub = rospy.Publisher(
        #     "/commands/motor/duty_cycle", Float64, queue_size=10
        # )

        # ROS Subscribers
        rospy.Subscriber(
            "/global_path/optimal_trajectory_wpnt", Marker, self.trajectory_callback
        )
        rospy.Subscriber(
            "/odom", Odometry, self.odom_callback
        )
        # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)

        # Vehicle Parameters
        self.dt = 0.1  # Time step (s)
        self.N = 4  # Prediction horizon
        self.wheelbase_length = 0.3302  # Wheelbase length (m)
        self.max_steer = np.radians(30)  # Maximum steering angle (rad)
        self.max_speed = 10  # Maximum speed (m/s)

        # State Variables
        self.current_pose = None
        self.yaw = 0.0
        self.trajectory = None
        self.current_speed = 0.0

    def trajectory_callback(self, msg):
        """웨이포인트를 저장"""
        self.trajectory = np.array([[p.x, p.y] for p in msg.points])

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

    def get_horizon_waypoints(self, waypoints, x0):
        """차량의 현재 위치를 기준으로 Horizon 웨이포인트를 동적으로 추출"""
        current_position = x0[:2]

        # 웨이포인트와의 거리 계산
        distances = np.linalg.norm(waypoints - current_position, axis=1)

        # 가장 가까운 웨이포인트 찾기
        closest_idx = np.argmin(distances)

        # Horizon만큼의 웨이포인트 반환
        return waypoints[closest_idx : closest_idx + self.N + 1]

    def solve_mpc(self, x0, waypoints):
        """
        Solve the MPC optimization problem using CasADi.
        """
        # Define optimization variables
        x = ca.MX.sym("x", 4, self.N + 1)  # States: [x, y, yaw, speed]
        u = ca.MX.sym("u", 2, self.N)  # Controls: [steering_angle, acceleration]
        # Define parameters (initial state, waypoints)
        P = ca.MX.sym("P", 4 + 2 * self.N)  # Initial state + waypoints

        # Objective and constraints
        obj = 0
        g = []

        Q = np.diag([0.5, 0.5, 2.0, 1.0])  # State cost
        R = np.diag([0.5, 0.01])  # Control cost

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
            state_error = x[0:2, k] - target
            obj += ca.mtimes([state_error.T, Q[0:2, 0:2], state_error])
            obj += ca.mtimes([u[:, k].T, R, u[:, k]])

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
            waypoints = self.get_horizon_waypoints(self.trajectory, x0)
            if len(waypoints) < self.N + 1:
                rospy.logwarn("Horizon 웨이포인트가 부족합니다.")
                self.rate.sleep()
                continue

            # MPC 최적화 문제 풀이
            control_input = self.solve_mpc(x0, waypoints)
            if control_input is None:
                self.rate.sleep()
                continue

            # 제어 입력 추출
                # Extract individual control inputs
            steering_angle = float(control_input[0])
            acceleration = float(control_input[1])

            self.current_speed += acceleration * self.dt

            # ROS 메시지로 제어 입력 발행
            # steer_msg = Float64()
            # speed_msg = Float64()
            # steer_msg.data = 0.5 - steering_angle  # 서보 범위에 맞게 변환
            # speed_msg.data = self.current_speed

            # self.steer_pub.publish(steer_msg)
            # self.speed_pub.publish(speed_msg)

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
