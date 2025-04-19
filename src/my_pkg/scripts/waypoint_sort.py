import numpy as np


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


# 파일로부터 좌표 불러오기
file_path = "/home/patrick/Downloads/red_line_coordinates.txt"
waypoints = np.loadtxt(file_path, delimiter=",")

# 좌표 정렬
sorted_waypoints = sort_waypoints_clockwise(waypoints)

# 결과를 새로운 txt 파일로 저장
output_file_path = "/home/patrick/Downloads/red_line_coordinates.txt"
np.savetxt(output_file_path, sorted_waypoints, delimiter=",")

print(f"정렬된 좌표가 {output_file_path}에 저장되었습니다.")
