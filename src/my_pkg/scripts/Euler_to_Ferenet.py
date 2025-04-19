import numpy as np
from scipy.spatial import KDTree


# 파일에서 좌표 읽기
def load_coordinates(file_path):
    coordinates = []
    with open(file_path, "r") as file:
        for line in file:
            x, y = map(int, line.strip().split(","))
            coordinates.append((x, y))
    return np.array(coordinates)


# 경로 순서 정렬
def sort_path(coordinates):
    sorted_coords = [coordinates[0]]  # 첫 번째 점을 기준으로 시작
    remaining_coords = coordinates[1:].tolist()  # 나머지 좌표들

    while remaining_coords:
        last_point = sorted_coords[-1]
        # KDTree를 이용하여 가장 가까운 좌표를 찾음
        tree = KDTree(remaining_coords)
        _, index = tree.query(last_point)
        closest_point = remaining_coords.pop(index)
        sorted_coords.append(closest_point)

    return np.array(sorted_coords)


# 프레넷 경로 계산: 각 점에서 이전 점까지의 거리를 누적하여 s값 계산
def compute_frenet_path(coordinates):
    s = np.zeros(len(coordinates))
    for i in range(1, len(coordinates)):
        dx = coordinates[i, 0] - coordinates[i - 1, 0]
        dy = coordinates[i, 1] - coordinates[i - 1, 1]
        s[i] = s[i - 1] + np.sqrt(dx**2 + dy**2)
    return s


# 프레넷 좌표계로 변환 후 텍스트 파일로 저장
def convert_to_frenet_and_save(file_path, output_file):
    coordinates = load_coordinates(file_path)
    sorted_coords = sort_path(coordinates)  # 좌표 정렬
    s = compute_frenet_path(sorted_coords)

    # 프레넷 좌표는 s와 d로 표현되며, 여기서는 d는 0으로 가정
    frenet_coords = np.column_stack((s, np.zeros_like(s)))

    # 결과를 텍스트 파일로 저장
    with open(output_file, "w") as f:
        for s_val, d_val in frenet_coords:
            f.write(f"{s_val:.2f}, {d_val:.2f}\n")


# 파일 경로
input_file = "/home/patrick/Downloads/red_line_coordinates.txt"
output_file = "/home/patrick/frenet_coordinates.txt"

# 프레넷 좌표 변환 및 저장
convert_to_frenet_and_save(input_file, output_file)

print(f"Frenet coordinates have been saved to {output_file}")
