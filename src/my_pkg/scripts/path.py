import cv2
import cv2.ximgproc
import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt


def skeletonize(img):
    img = (img == 255).astype(np.uint8)
    skeleton = np.zeros_like(img)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)) #

    while True:
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skeleton = cv2.bitwise_or(skeleton, temp)
        img = eroded.copy()

        if cv2.countNonZero(img) == 0:
            break

    return skeleton


# SLAM으로 생성된 지도를 불러옵니다 (예: 'slam_map.png')
slam_map = cv2.imread("/home/patrick/f1_ws/src/f1tenth_simulator/maps/berlin.png", cv2.IMREAD_GRAYSCALE)

# 점유 그리드를 이진화합니다
_, binary_map = cv2.threshold(slam_map,220, 255, cv2.THRESH_BINARY)

# 모폴로지 오픈 필터를 적용하여 노이즈를 제거합니다
kernel = np.ones((3, 3), np.uint8)
opened_map = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, kernel)

# 중심선을 추출합니다 (모폴로지 스켈레톤 방법 사용)
# skeleton = skeletonize(opened_map)
skeleton = skeletonize(opened_map)
# 중심선을 매끄럽게 하기 위해 Savitzky-Golay 필터를 적용합니다
# 중심선의 좌표를 추출합니다
y_coords, x_coords = np.nonzero(skeleton)

# 좌표를 s 값에 따라 정렬합니다 (s는 거리 또는 인덱스)
sorted_indices = np.argsort(y_coords)
sorted_x = x_coords[sorted_indices]
sorted_y = y_coords[sorted_indices]

# Savitzky-Golay 필터를 적용합니다
window_length = 51  # 윈도우 크기, 홀수여야 합니다 (조정 필요)
polyorder = 3  # 다항식 차수 (조정 필요)
smoothed_x = savgol_filter(sorted_x, window_length, polyorder)
smoothed_y = savgol_filter(sorted_y, window_length, polyorder)

# 컬러 맵 생성
color_slam_map = cv2.cvtColor(slam_map, cv2.COLOR_GRAY2BGR)

# 원래 그림 위에 중심선을 빨간색으로 그리기
for x, y in zip(sorted_x, sorted_y):
    color_slam_map[y, x] = [0, 0, 255]  # 빨간색

# 매끄럽게 된 중심선을 원래 그림 위에 그리기
for x, y in zip(sorted_x, smoothed_y.astype(int)):
    color_slam_map[y, x] = [0, 255, 0]  # 녹색 (매끄럽게 된 중심선)

# 결과를 시각화합니다
plt.figure(figsize=(12, 6))
plt.title("SLAM Map with Centerline")
plt.imshow(color_slam_map)
plt.show()

# 결과를 파일로 저장합니다
cv2.imwrite("slam_map_with_centerline.png", color_slam_map)
waypoints = np.column_stack((sorted_x, smoothed_y.astype(int)))
np.savetxt("waypoints_berlin.txt", waypoints, delimiter=",")
