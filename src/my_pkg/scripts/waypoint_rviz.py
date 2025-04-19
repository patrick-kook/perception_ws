#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray, Pose
import numpy as np
from std_msgs.msg import ColorRGBA
from scipy.signal import savgol_filter


def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def sort_waypoints_clockwise(waypoints):
    sorted_waypoints = []
    current_point = waypoints[0]  # 시작점
    sorted_waypoints.append(current_point)
    waypoints = np.delete(waypoints, 0, axis=0)  # 시작점을 제거

    while len(waypoints) > 0:
        distances = [distance(current_point, point) for point in waypoints]
        closest_idx = np.argmin(distances)
        current_point = waypoints[closest_idx]
        sorted_waypoints.append(current_point)
        waypoints = np.delete(waypoints, closest_idx, axis=0)  # 가장 가까운 점 제거

    return np.array(sorted_waypoints)


class WaypointsMarkerPublisher:
    def __init__(self):
        rospy.init_node("waypoints_marker_publisher", anonymous=True)
        self.marker_pub = rospy.Publisher(
            "/waypoints_marker", Marker, queue_size=10
        )
        self.posearray_pub = rospy.Publisher(
            "/waypoints", PoseArray, queue_size=10
        )

        # 파일 경로 설정
        self.file_path = "/home/patrick/Downloads/0923.txt"
        self.waypoints = np.loadtxt(self.file_path, delimiter=",")

        # 시계 방향으로 정렬
        self.waypoints = sort_waypoints_clockwise(self.waypoints)
        # Savitzky-Golay 필터를 적용합니다

        window_length = 91  # 윈도우 크기, 홀수여야 합니다 (조정 필요)
        polyorder = 2  # 다항식 차수 (조정 필요)
        smoothed_x = savgol_filter(self.waypoints[:, 0], window_length, polyorder)
        smoothed_y = savgol_filter(self.waypoints[:, 1], window_length, polyorder)

        self.waypoints[:, 0] = smoothed_x
        self.waypoints[:, 1] = smoothed_y

        self.image_height = 600  # 이미지 높이
        self.resolution = 0.08000
        self.origin = [-51.224998, -51.224998, 0.000000]
        self.rate = rospy.Rate(1000000000000000000000000000000)  # 1 Hz

    def pixel_to_world(self, pixel_x, pixel_y):
        world_x = pixel_x * self.resolution + self.origin[0]
        world_y = pixel_y * self.resolution + self.origin[1]

        # Savitzky-Golay 필터를 적용합니다

        return world_x, world_y

    def calculate_curvature(self, p1, p2, p3):
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        if a == 0 or b == 0 or c == 0:
            return 0  # 세 점이 겹치면 곡률은 0

        # 외접원의 반지름 계산
        s = (a + b + c) / 2  # 삼각형 둘레의 절반
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))

        if area == 0:
            return 0

        radius = (a * b * c) / (4 * area)
        curvature = 1 / radius  # 곡률은 반지름의 역수
        return curvature

    def calculate_average_curvature(self, segment):
        curvatures = []
        for i in range(1, len(segment) - 1):
            curvatures.append(
                self.calculate_curvature(segment[i - 1], segment[i], segment[i + 1])
            )
        if len(curvatures) == 0:
            return 0
        return np.mean(curvatures)

    def split_segments(self, waypoints, segment_length):
        segments = []
        for i in range(0, len(waypoints) - segment_length, segment_length):
            segment = waypoints[i : i + segment_length]
            segments.append(segment)
        return segments

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # 선으로 표시
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05  # 선의 두께

        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        # 웨이포인트를 일정 구간으로 나누기 (섹터 크기 설정)
        segment_length =10
        segments = self.split_segments(self.waypoints, segment_length)

        # 각 구간에서 평균 곡률 계산
        for segment in segments:
            average_curvature = self.calculate_average_curvature(segment)
            max_curvature = 0.025 # 예시로 최대 곡률을 설정
            min_curvature = 0.0  # 예시로 최소 곡률을 설정

            # 곡률에 따른 색상 계산 (빨간색에서 녹색으로 선형 보간)
            t = (average_curvature - min_curvature) / (max_curvature - min_curvature)
            t = np.clip(t, 0.0, 1.0)  # t 값이 0~1 사이에 있도록 제한
            interpolated_color = np.array([t, 1.0 - t, 0.0])  # 빨간색에서 녹색으로 전환

            for waypoint in segment:
                world_x, world_y = self.pixel_to_world(waypoint[0], waypoint[1])

                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = 0
                marker.points.append(point)

                color = ColorRGBA()
                color.r, color.g, color.b = interpolated_color
                color.a = 1.0
                marker.colors.append(color)

                pose = Pose()
                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = 0
                pose_array.poses.append(pose)

        while not rospy.is_shutdown():
            self.marker_pub.publish(marker)
            self.posearray_pub.publish(pose_array)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        wp = WaypointsMarkerPublisher()
        wp.publish_markers()
    except rospy.ROSInterruptException:
        pass
