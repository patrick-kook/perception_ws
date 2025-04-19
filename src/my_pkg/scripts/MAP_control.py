#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import tf


class MAPController:
    def __init__(self):
        rospy.init_node("map_controller", anonymous=False)
        self.rate = rospy.Rate(20)  # 20 Hz

        # Publishers
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=10)

        # Subscribers
        rospy.Subscriber(
            "/global_path/optimal_trajectory", Marker, self.waypoints_callback
        )
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Parameters
        self.waypoints = None
        self.current_pose = None
        self.current_speed = 0
        self.yaw = 0.0
        self.wheelbase_length = 0.3302  # Vehicle wheelbase length in meters
        self.max_steering_angle = np.radians(30)  # Maximum steering angle in radians
        self.max_acceleration = 2.0  # Maximum acceleration (m/s^2)
        self.min_speed = 0.5  # Minimum speed (m/s)
        self.max_speed = 5.0  # Maximum speed (m/s)

    def waypoints_callback(self, msg):
        self.waypoints = np.array([[pose.x, pose.y] for pose in msg.points])

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

    def compute_steering_and_acceleration(self, target_point):
        if target_point is None or self.current_pose is None:
            return 0.0, 0.0

        # Compute errors
        dx = target_point[0] - self.current_pose.position.x
        dy = target_point[1] - self.current_pose.position.y

        # Transform target point to the vehicle's local coordinate frame
        local_x = np.cos(self.yaw) * dx + np.sin(self.yaw) * dy
        local_y = -np.sin(self.yaw) * dx + np.cos(self.yaw) * dy

        # Compute curvature (kappa) for the path
        curvature = 2 * local_y / (local_x**2 + local_y**2)

        # Compute steering angle using curvature and wheelbase
        steering_angle = np.arctan(curvature * self.wheelbase_length)

        # Clamp the steering angle  
        steering_angle = np.clip(
            steering_angle, -self.max_steering_angle, self.max_steering_angle
        )

        # Compute desired speed based on curvature
        target_speed = self.max_speed * (1 - abs(curvature))
        target_speed = max(self.min_speed, min(target_speed, self.max_speed))

        # Compute acceleration
        acceleration = (target_speed - self.current_speed) * self.max_acceleration

        return steering_angle, acceleration

    def get_target_point(self):
        if self.waypoints is None or self.current_pose is None:
            return None

        closest_dist = float("inf")
        target_point = None

        for point in self.waypoints:
            dist = np.linalg.norm(
                np.array(
                    [
                        point[0] - self.current_pose.position.x,
                        point[1] - self.current_pose.position.y,
                    ]
                )
            )

            if dist < closest_dist:
                closest_dist = dist
                target_point = point

        self.publish_marker(target_point)
        return target_point

    def publish_marker(self, point):
        if point is None:
            return

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
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def run(self):
        while not rospy.is_shutdown():
            target_point = self.get_target_point()

            if target_point is not None:
                steering_angle, acceleration = self.compute_steering_and_acceleration(
                    target_point
                )

                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = steering_angle
                drive_msg.drive.speed = (
                    self.current_speed + acceleration / self.rate.sleep_dur.to_sec()
                )

                self.drive_pub.publish(drive_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        controller = MAPController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
