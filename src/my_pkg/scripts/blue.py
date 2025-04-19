#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA  # ColorRGBA를 사용하기 위해 import

# 이미지 경로 및 파라미터 설정
image_path = "/home/patrick/Downloads/blue.jpeg"
SCALE_FACTOR = 0.0101
SAMPLE_DISTANCE = 1

# 회전 각도 및 이동 값 (미터)
ROTATION_ANGLE = 347  # 시계 방향으로 45도 회전
TRANSLATE_X = -9.45    # X축으로 1.0m 이동
TRANSLATE_Y = -4.5     # Y축으로 0.5m 이동

# 추출된 포인트들을 일정 간격으로 샘플링하는 함수
def interpolate_points(points, distance):
    sampled_points = []

    for i in range(len(points) - 1):
        p1 = points[i]  
        p2 = points[i + 1]

        x1, y1 = p1
        x2, y2 = p2

        line_length = math.hypot(x2 - x1, y2 - y1)
        num_samples = max(int(line_length / distance), 1)

        for j in range(num_samples):
            t = j / num_samples
            x = (1 - t) * x1 + t * x2
            y = (1 - t) * y1 + t * y2
            sampled_points.append((x * SCALE_FACTOR, y * SCALE_FACTOR))

    return sampled_points

# 이미지에서 특정 색상의 컨투어를 추출하고 샘플링하는 함수
def extract_and_sample_contours(image_path, color):
    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if color == "black":
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 75, 255, cv2.THRESH_BINARY_INV)
    elif color == "red":
        # Red color range
        lower_red1 = np.array([0, 10, 20])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([140, 10, 20])
        upper_red2 = np.array([200, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == "green":
        # Green color range (adjust values as needed)
        lower_green = np.array([35, 20, 30])  # Lower bound for green HSV range
        upper_green = np.array([85, 255, 255])  # Upper bound for green HSV range
        mask = cv2.inRange(hsv, lower_green, upper_green)
    elif color == "yellow":
        # Yellow color range (adjust values as needed)
        lower_yellow = np.array([20, 30, 20])  # Lower bound for yellow HSV range
        upper_yellow = np.array([60, 255, 255])  # Upper bound for yellow HSV range
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif color == "blue":
        # Blue color range (adjust values as needed)
        lower_blue = np.array([100, 150, 0])  # Lower bound for blue HSV range
        upper_blue = np.array([140, 255, 255])  # Upper bound for blue HSV range
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
    else:
        return []

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    all_sampled_points = []
    for contour in contours:
        simplified_contour = cv2.approxPolyDP(contour, epsilon=5, closed=True)
        points = [(p[0][0], p[0][1]) for p in simplified_contour]
        sampled_points = interpolate_points(points, SAMPLE_DISTANCE)
        all_sampled_points.extend(sampled_points)

    return all_sampled_points

# 포인트들의 중심을 계산하는 함수
def calculate_center(points):
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    center_x = sum(x_coords) / len(x_coords)
    center_y = sum(y_coords) / len(y_coords)
    return center_x, center_y

# 포인트들을 회전 및 이동시키는 함수
def transform_points(points, center_x, center_y, angle, translate_x, translate_y):
    angle_rad = math.radians(angle)
    transformed_points = []

    for x, y in points:
        # 중심점 기준으로 이동
        x -= center_x
        y -= center_y

        # 회전 변환
        new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)

        # 중심점으로 다시 이동하고, 추가 이동 적용
        new_x += center_x + translate_x
        new_y += center_y + translate_y

        transformed_points.append((new_x, new_y))

    return transformed_points

# 포인트들과 색상 정보를 /trajectory로 퍼블리시하는 함수
def publish_points(track_points, red_trajectory_points, green_trajectory_points, yellow_trajectory_points, blue_trajectory_points):
    rospy.init_node("track_and_trajectory_publisher")
    track_pub = rospy.Publisher("/track", Marker, queue_size=10)  # /track 토픽에 검정색 포인트 퍼블리시
    traj_pub = rospy.Publisher("/trajectory", Marker, queue_size=10)  # /trajectory 토픽에 빨강, 초록, 노랑, 파랑 포인트 퍼블리시

    # Track Marker 객체 (검정색 트랙)
    track_marker = Marker()
    track_marker.header.frame_id = "map"
    track_marker.header.stamp = rospy.Time.now()
    track_marker.ns = "track_points"
    track_marker.id = 0
    track_marker.type = Marker.POINTS
    track_marker.action = Marker.ADD
    track_marker.scale.x = 0.05
    track_marker.scale.y = 0.05
    track_marker.color.a = 1.0

    # Trajectory Marker 객체 (빨간색, 초록색, 노란색, 파란색 경로)
    traj_marker = Marker()
    traj_marker.header.frame_id = "map"
    traj_marker.header.stamp = rospy.Time.now()
    traj_marker.ns = "trajectory_points"
    traj_marker.id = 1
    traj_marker.type = Marker.POINTS
    traj_marker.action = Marker.ADD
    traj_marker.scale.x = 0.05
    traj_marker.scale.y = 0.05
    traj_marker.color.a = 1.0

    # Track 포인트 (검정색)
    for point in track_points:
        x, y = point
        z = 0  # z 좌표는 0으로 설정
        pt = Point(x=x, y=y, z=z)
        track_marker.points.append(pt)

        # 검정색
        color = ColorRGBA()
        color.r = 0.0
        color.g = 0.0
        color.b = 0.0  # 검정색
        color.a = 1.0
        track_marker.colors.append(color)

    # Trajectory 포인트 (빨강, 초록색, 노란색, 파란색)
    for point in red_trajectory_points:
        x, y = point
        z = 0  # z 좌표는 0으로 설정
        pt = Point(x=x, y=y, z=z)
        traj_marker.points.append(pt)

        # 빨간색
        color = ColorRGBA()
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0  # 빨강색
        color.a = 1.0
        traj_marker.colors.append(color)

    for point in green_trajectory_points:
        x, y = point
        z = 0  # z 좌표는 0으로 설정
        pt = Point(x=x, y=y, z=z)
        traj_marker.points.append(pt)

        # 초록색
        color = ColorRGBA()
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0  # 초록색
        color.a = 1.0
        traj_marker.colors.append(color)

    for point in yellow_trajectory_points:
        x, y = point
        z = 0  # z 좌표는 0으로 설정
        pt = Point(x=x, y=y, z=z)
        traj_marker.points.append(pt)

        # 노란색
        color = ColorRGBA()
        color.r = 1.0
        color.g = 1.0
        color.b = 0.0  # 노란색
        color.a = 1.0
        traj_marker.colors.append(color)

    for point in blue_trajectory_points:
        x, y = point
        z = 0  # z 좌표는 0으로 설정
        pt = Point(x=x, y=y, z=z)
        traj_marker.points.append(pt)

        # 파란색
        color = ColorRGBA()
        color.r = 0.0
        color.g = 0.0
        color.b = 1.0  # 파란색
        color.a = 1.0
        traj_marker.colors.append(color)

    # 퍼블리시
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        track_pub.publish(track_marker)  # 검정색 트랙 퍼블리시
        traj_pub.publish(traj_marker)    # 빨강, 초록, 노랑, 파랑 경로 퍼블리시
        rate.sleep()

if __name__ == "__main__":
    # 색상에 따라 포인트 추출
    track_points = extract_and_sample_contours(image_path, "black")
    red_trajectory_points = extract_and_sample_contours(image_path, "red")
    green_trajectory_points = extract_and_sample_contours(image_path, "green")
    yellow_trajectory_points = extract_and_sample_contours(image_path, "yellow")
    blue_trajectory_points = extract_and_sample_contours(image_path, "blue")

    # 트랙과 경로 포인트들의 중심 계산
    track_center_x, track_center_y = calculate_center(track_points)
    traj_center_x, traj_center_y = calculate_center(red_trajectory_points + green_trajectory_points + yellow_trajectory_points + blue_trajectory_points)

    # 회전 및 이동 변환 적용
    track_points = transform_points(track_points, track_center_x, track_center_y, ROTATION_ANGLE, TRANSLATE_X, TRANSLATE_Y)
    red_trajectory_points = transform_points(red_trajectory_points, track_center_x, track_center_y, ROTATION_ANGLE, TRANSLATE_X, TRANSLATE_Y)
    green_trajectory_points = transform_points(green_trajectory_points, track_center_x, track_center_y, ROTATION_ANGLE, TRANSLATE_X, TRANSLATE_Y)
    yellow_trajectory_points = transform_points(yellow_trajectory_points, track_center_x, track_center_y, ROTATION_ANGLE, TRANSLATE_X, TRANSLATE_Y)
    blue_trajectory_points = transform_points(blue_trajectory_points, track_center_x, track_center_y, ROTATION_ANGLE, TRANSLATE_X, TRANSLATE_Y)

    # 경로와 트랙 퍼블리시
    publish_points(track_points, red_trajectory_points, green_trajectory_points, yellow_trajectory_points, blue_trajectory_points)
