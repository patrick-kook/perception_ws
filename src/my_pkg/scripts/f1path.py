#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseArray, Pose
import numpy as np



class WaypointsMarkerPublisher:
    def __init__(self):
        rospy.init_node("waypoints_marker_publisher", anonymous=True)

        # 원본 좌표, 확대 좌표, 축소 좌표를 각각 다른 토픽으로 발행
        self.marker_pub = rospy.Publisher("/waypoints_marker", Marker, queue_size=10)
        self.scaled_marker_pub = rospy.Publisher(
            "/scaled_waypoints_marker", Marker, queue_size=10
        )
        self.shrunk_marker_pub = rospy.Publisher(
            "/shrunk_waypoints_marker", Marker, queue_size=10
        )

        # PoseArray 발행
        self.posearray_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=10)
        self.scaled_posearray_pub = rospy.Publisher(
            "/scaled_waypoints", PoseArray, queue_size=10
        )
        self.shrunk_posearray_pub = rospy.Publisher(
            "/shrunk_waypoints", PoseArray, queue_size=10
        )

        # 파일 경로 설정
        self.file_path = (
            "/home/patrick/f1_ws/src/f1tenth_simulator/scripts/waypoints_berlin.txt"
        )
        self.waypoints = np.loadtxt(self.file_path, delimiter=",")

        # 기본 설정 값
        self.image_height = 600  # 이미지 높이
        self.resolution = 0.050000
        self.origin = [-11.606540, -26.520793, 0.000000]
        x_max = np.max(self.waypoints[:, 0])
        x_min = np.min(self.waypoints[:, 0])

        # y좌표들의 최댓값과 최솟값
        y_max = np.max(self.waypoints[:, 1])
        y_min = np.min(self.waypoints[:, 1])

        # 각각의 중간값 (최댓값과 최솟값의 중간값)
        x_mid = (x_max + x_min) / 2
        y_mid = (y_max + y_min) / 2
        print(x_mid," ", y_mid)

        # 중간값으로 새로운 좌표 생성
        self.center = np.array([x_mid,y_mid])  # 확대/축소 기준이 될 중심점
        self.scale_factor = 1.1  # 확대 비율
        self.shrink_factor = 0.9  # 축소 비율
        self.rate = rospy.Rate(1)  # 1 Hz

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        픽셀 좌표를 월드 좌표로 변환
        """
        world_x = pixel_x * self.resolution + self.origin[0]
        world_y = (self.image_height - pixel_y) * self.resolution + self.origin[1]
        return world_x, world_y

    def scale_points(self, points, center, scale_factor):
        """
        좌표를 중심점을 기준으로 확대 또는 축소
        """
        center_x = center[0] * self.resolution + self.origin[0]
        center_y = (self.image_height - center[1]) * self.resolution + self.origin[1]
        center=np.array([center_x, center_y])
        translated_points = points - center
        scaled_points = translated_points * scale_factor
        transformed_points = scaled_points + center
        return transformed_points

    def publish_markers(self):
        # 원본 좌표
        marker = self.create_marker("map", 0, [1.0, 0.0, 0.0])  # 빨간색
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        # 확대 좌표
        scaled_marker = self.create_marker("map", 1, [0.0, 1.0, 0.0])  # 초록색
        scaled_pose_array = PoseArray()
        scaled_pose_array.header.frame_id = "map"
        scaled_pose_array.header.stamp = rospy.Time.now()

        # 축소 좌표
        shrunk_marker = self.create_marker("map", 2, [0.0, 0.0, 1.0])  # 파란색
        shrunk_pose_array = PoseArray()
        shrunk_pose_array.header.frame_id = "map"
        shrunk_pose_array.header.stamp = rospy.Time.now()

        # 모든 좌표 처리
        for pixel_x, pixel_y in self.waypoints:
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
            original_point = np.array([world_x, world_y])

            # 원본 좌표 추가
            self.add_point_to_marker(marker, pose_array, original_point)

            # 확대된 좌표 추가
            scaled_point = self.scale_points(
                original_point, self.center, self.scale_factor
            )
            self.add_point_to_marker(scaled_marker, scaled_pose_array, scaled_point)

            # 축소된 좌표 추가
            shrunk_point = self.scale_points(
                original_point, self.center, self.shrink_factor
            )
            self.add_point_to_marker(shrunk_marker, shrunk_pose_array, shrunk_point)

        # ROS 루프에서 마커 발행
        while not rospy.is_shutdown():
            self.marker_pub.publish(marker)
            self.posearray_pub.publish(pose_array)
            self.scaled_marker_pub.publish(scaled_marker)
            self.scaled_posearray_pub.publish(scaled_pose_array)
            self.shrunk_marker_pub.publish(shrunk_marker)
            self.shrunk_posearray_pub.publish(shrunk_pose_array)
            self.rate.sleep()

    def create_marker(self, frame_id, marker_id, color):
        """
        기본 마커 생성
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 점의 크기
        marker.scale.y = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def add_point_to_marker(self, marker, pose_array, point):
        """
        마커와 PoseArray에 좌표를 추가
        """
        # 마커에 점 추가
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = 0
        marker.points.append(p)

        # PoseArray에 Pose 추가
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = 0
        pose_array.poses.append(pose)


if __name__ == "__main__":
    try:
        wp = WaypointsMarkerPublisher()
        wp.publish_markers()
    except rospy.ROSInterruptException:
        pass
