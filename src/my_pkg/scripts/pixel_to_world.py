import numpy as np
import pandas as pd

# Define file paths and parameters
file_path = "/home/patrick/trajectory_generator/f1tenth-racing-stack-ICRA22/trajectory_generator/outputs/1008/test1_bound.txt"
resolution = 0.05
origin = np.array([-50.924998, 21.224998, 0.000000]) 
image_height = 600  # Origin offset

# Load CSV file with pixel coordinates
df_pixels = pd.read_csv(file_path)
df_pixels.columns = [
    "# x_px",
    "y_px",
]  # Assumes pixel coordinates are in columns named x_px, y_px

# Convert pixel coordinates to XY coordinates using resolution and origin
df_pixels["# x_m"] = df_pixels["# x_px"] * resolution + origin[0]
df_pixels["y_m"] = (image_height-df_pixels["y_px"]) * resolution + origin[1]

# Define obstacle and path widths
half_width = 0.2  # Adjustable half-width for path lane
obs_width = 0.5  # Adjustable width for obstacle avoidance lanes

# Calculate left (inner) and right (outer) lanes based on angle or central path
left_lane_x = df_pixels["# x_m"] - half_width
left_lane_y = df_pixels["y_m"]

right_lane_x = df_pixels["# x_m"] + half_width
right_lane_y = df_pixels["y_m"]

# Obstacle lanes (inner and outer) with a wider offset
obs_inner_x = df_pixels["# x_m"] - obs_width
obs_inner_y = df_pixels["y_m"]

obs_outer_x = df_pixels["# x_m"] + obs_width
obs_outer_y = df_pixels["y_m"]

# Save to CSV files
inner_lane = pd.DataFrame({"# x_m": left_lane_x, "y_m": left_lane_y})
inner_lane.to_csv(f"{file_path}scaled_inner.csv", index=False)

outer_lane = pd.DataFrame({"# x_m": right_lane_x, "y_m": right_lane_y})
outer_lane.to_csv(f"{file_path}scaled_outer.csv", index=False)

obs_inner_lane = pd.DataFrame({"# x_m": obs_inner_x, "y_m": obs_inner_y})
obs_inner_lane.to_csv(f"{file_path}scaled_obs_inner.csv", index=False)

obs_outer_lane = pd.DataFrame({"# x_m": obs_outer_x, "y_m": obs_outer_y})
obs_outer_lane.to_csv(f"{file_path}scaled_obs_outer.csv", index=False)

print(
    "Converted and scaled inner, outer, and obstacle lanes have been saved to CSV files."
)
