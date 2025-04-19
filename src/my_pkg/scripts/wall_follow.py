#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class WallFollower:
    def __init__(self):
        rospy.init_node("wall_follower", anonymous=True)

        # AckermannDrive 메시지를 퍼블리시할 퍼블리셔 설정
        self.cmd_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # 라이다 데이터를 구독할 서브스크라이버 설정
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.rate = rospy.Rate(10)  # 10Hz

        # Pure Pursuit 알고리즘의 파라미터 설정
        self.lookahead_distance = 1.0  # Lookahead 거리
        self.max_speed = 1.0  # 최대 속도
        self.min_speed = 0.1  # 최소 속도

    def scan_callback(self, msg):
        # 라이다 데이터에서 필요한 정보 추출
        ranges = np.array(msg.ranges)
        print(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # 라이다 데이터에서 중앙의 거리 계산
        mid_index = len(ranges) // 2
        distance = ranges[mid_index]

        # Pure Pursuit 알고리즘에 기반한 제어 명령 생성
        drive_cmd = AckermannDriveStamped()

        # 중앙 벽을 따라가도록 속도 및 조향 각도 계산
        drive_cmd.drive.speed = self.calculate_speed(distance)
        drive_cmd.drive.steering_angle = self.calculate_steering_angle(
            ranges, angle_min, angle_increment
        )

        # 제어 명령 퍼블리시
        self.cmd_pub.publish(drive_cmd)

    def calculate_speed(self, distance):
        # 거리 기반으로 속도를 조절
        if distance < 1.0:
            return self.min_speed
        else:
            return min(self.max_speed, distance / 2.0)

    def calculate_steering_angle(self, ranges, angle_min, angle_increment):
        # Pure Pursuit에서의 조향 각도 계산
        angles = np.arange(
            angle_min, angle_min + len(ranges) * angle_increment, angle_increment
        )
        min_index = np.argmin(ranges)
        target_angle = angles[min_index]

        # 조향 각도는 벽의 방향에 따라 조정
        steering_angle = np.arctan2(np.sin(target_angle), np.cos(target_angle))
        return steering_angle

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    wall_follower = WallFollower()
    wall_follower.run()
