import numpy as np
import pandas as pd

# CSV 파일 읽기 (경로는 실제 경로로 수정)
file_path = "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/finalv2/traj_race_cl.csv"

# CSV 파일을 읽어서 df_cleaned 데이터프레임 생성
df_cleaned = pd.read_csv(file_path, delimiter=";", skiprows=2)

# 열 이름 설정
df_cleaned.columns = ["s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "vy_mps"]

# 로봇의 반 너비 (0.1m)
half_width = 0.2  # 원하는 너비로 설정 가능
obs_width = 0.5

# 경로의 각 좌표에서 수직 방향으로 이동하기 위한 벡터 계산
# 좌측 (inner) 라인: 중앙 경로의 왼쪽으로 half_width만큼 떨어진 좌표 계산
left_lane_x = df_cleaned["x_m"] - half_width * np.cos(df_cleaned["psi_rad"])
left_lane_y = df_cleaned["y_m"] - half_width * np.sin(df_cleaned["psi_rad"])

# 우측 (outer) 라인: 중앙 경로의 오른쪽으로 half_width만큼 떨어진 좌표 계산
right_lane_x = df_cleaned["x_m"] + half_width * np.cos(df_cleaned["psi_rad"])
right_lane_y = df_cleaned["y_m"] + half_width * np.sin(df_cleaned["psi_rad"])

# obtacle inner lane
obs_lane_x = df_cleaned["x_m"] - obs_width * np.cos(df_cleaned["psi_rad"])
obs_lane_y = df_cleaned["y_m"] - obs_width * np.sin(df_cleaned["psi_rad"])

obs_outer_lane_x = df_cleaned["x_m"] + obs_width * np.cos(df_cleaned["psi_rad"])
obs_outer_lane_y = df_cleaned["y_m"] + obs_width * np.sin(df_cleaned["psi_rad"])


# 좌측 (inner) 라인을 CSV로 저장
inner_lane = pd.DataFrame({"# x_m": left_lane_x, "y_m": left_lane_y,})
inner_lane.to_csv(
    "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/finalv2_in/inner.csv",
    index=False,
)

# 우측 (outer) 라인을 CSV로 저장
outer_lane = pd.DataFrame({"# x_m": right_lane_x, "y_m": right_lane_y})
outer_lane.to_csv(
    "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/finalv2_in/outer.csv",
    index=False,
)

obstacle_lane= pd.DataFrame({"# x_m": obs_lane_x, "y_m": obs_lane_y})
obstacle_lane.to_csv(
    "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/finalv2_in/obs_inner.csv",
    index=False,
)

obstacle_outer_lane = pd.DataFrame({"# x_m": obs_outer_lane_x, "y_m": obs_outer_lane_y})
obstacle_outer_lane.to_csv(
    "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/finalv2_in/obs_outer.csv",
    index=False,
)


print("Inner 및 Outer 라인이 각각 CSV 파일로 저장되었습니다.")
