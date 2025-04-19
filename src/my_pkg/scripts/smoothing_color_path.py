import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import helper_funcs_glob

# 경로 데이터 불러오기 (기존 centerline.csv 경로 또는 데이터프레임 사용)
file_path = "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/csv/1001/centerline.csv"
centerline_df = pd.read_csv(file_path)

x = centerline_df["#x_m"].values
y = centerline_df["y_m"].values

reftrack_imp = helper_funcs_glob.src.import_track.import_track(
    imp_opts=imp_opts,
    file_path=file_paths["track_file"],
    width_veh=pars["veh_params"]["width"],
)

# prep_track 함수를 사용하여 경로를 준비
(   reftrack_interp,
    normvec_normalized_interp,
    a_interp,
    coeffs_x_interp,
    coeffs_y_interp,
) = helper_funcs_glob.src.prep_track.prep_track(
    reftrack_imp=reftrack_imp,
    reg_smooth_opts=pars["reg_smooth_opts"],
    stepsize_opts=pars["stepsize_opts"],
    debug=debug,
    min_width=imp_opts["min_track_width"],
)

track = helper_funcs_glob.src.prep_track.prep_track(x, y)

# 스플라인 보간된 경로 좌표 가져오기
x_smooth = track["x_smooth"]
y_smooth = track["y_smooth"]

# 곡률 계산
dx, dy = np.gradient(x_smooth), np.gradient(y_smooth)
ddx, ddy = np.gradient(dx), np.gradient(dy)
curvature = np.abs(ddx * dy - dx * ddy) / (dx**2 + dy**2) ** 1.5

# 곡률을 0과 1 사이로 정규화
curvature_normalized = (curvature - curvature.min()) / (
    curvature.max() - curvature.min()
)

# 곡률에 따른 색상 RGBA 값 생성 (빨간색 -> 곡률 큼, 초록색 -> 곡률 작음)
colors = plt.cm.jet(1 - curvature_normalized)
rgba_values = np.array(colors[:, :4])  # RGBA 값만 추출 (R, G, B, A)

# 새로운 데이터프레임 생성 (X, Y 좌표와 RGBA 값 포함)
output_df = pd.DataFrame(
    {
        "x_m": x_smooth,
        "y_m": y_smooth,
        "R": rgba_values[:, 0],
        "G": rgba_values[:, 1],
        "B": rgba_values[:, 2],
        "A": rgba_values[:, 3],
    }
)

# 결과를 기존 파일 위치에 덮어쓰기 (CSV로 저장)
output_file_path = file_path  # 기존 파일에 덮어쓰기
output_df.to_csv(output_file_path, index=False)

print(f"파일이 성공적으로 {output_file_path}에 저장되었습니다.")
