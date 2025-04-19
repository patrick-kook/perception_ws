#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
import tf


class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publisher for Ackermann drive messages
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # Subscribers for waypoints and odometry
        rospy.Subscriber("/waypoints", PoseArray, self.waypoints_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Parameters
        self.waypoints = None
        self.base_lookahead_distance = 1# 기본 lookahead distance
        self.speed_factor = 0.7 # 속도에 따른 조정 인자
        self.wheelbase_length = 0.3302  # meters
        self.current_pose = None
        self.current_speed = 0

    def waypoints_callback(self, msg):
        self.waypoints = np.array(
            [[pose.position.x, pose.position.y] for pose in msg.poses]
        )

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_speed = msg.twist.twist.linear.x

    def get_target_point(self, lookahead_distance):
        # Find the lookahead point in the waypoints
        if self.waypoints is None or self.current_pose is None:
            return None

        transformed_waypoints = self.waypoints

        closest_dist = float("inf")
        target_point = None
        for point in transformed_waypoints:
            # 차량의 현재 위치로부터 각 웨이포인트까지의 거리를 계산합니다
            dist = np.linalg.norm(
                np.array(
                    [
                        point[0] - self.current_pose.position.x,
                        point[1] - self.current_pose.position.y,
                    ]
                )
            )
            if dist > self.base_lookahead_distance and dist < closest_dist:
                closest_dist = dist
                target_point = point
                print(target_point)

        return target_point

    def compute_steering_angle(self, target_point):
        # Compute the steering angle using the Pure Pursuit algorithm
        if target_point is None or self.current_pose is None:
            return 0.0

        # 차량의 현재 위치와 목표 지점 사이의 각도를 계산합니다
        alpha = np.arctan2(
            target_point[1] - self.current_pose.position.y,
            target_point[0] - self.current_pose.position.x,
        )
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

            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = steering_angle 
            drive_msg.drive.speed=0
            if steering_angle>0.3 or steering_angle<-0.3:
                drive_msg.drive.speed=1
            else:
                drive_msg.drive.speed = 2

            self.drive_pub.publish(drive_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
