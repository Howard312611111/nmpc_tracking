import rosbag
import numpy as np
from scipy.interpolate import interp1d

# === CONFIG 設定區 ===
ideal_distance = 540.0          # 期望距離 (公尺)
exclude_first_100s = 0       # 是否排除前 100 秒資料

# Function to compute Euclidean distance in the XY plane
def compute_distance(uav_x, uav_y, wamv_x, wamv_y):
    return np.sqrt((np.array(uav_x) - np.array(wamv_x))**2 + (np.array(uav_y) - np.array(wamv_y))**2)

# Load the bag file
bag_file = "Rework_judg2.bag"
bag = rosbag.Bag(bag_file, "r")

# Lists to store data
time_stamps_uav, uav0_x, uav0_y = [], [], []
time_stamps_wamv, wamv_x, wamv_y = [], [], []

# Read messages
for topic, msg, t in bag.read_messages(topics=["/uav0/base_pose_ground_truth", "/wamv/base_pose_ground_truth"]):
    if topic == "/uav0/base_pose_ground_truth":
        uav0_x.append(msg.pose.pose.position.x)
        uav0_y.append(msg.pose.pose.position.y)
        time_stamps_uav.append(t.to_sec())
    elif topic == "/wamv/base_pose_ground_truth":
        wamv_x.append(msg.pose.pose.position.x)
        wamv_y.append(msg.pose.pose.position.y)
        time_stamps_wamv.append(t.to_sec())

bag.close()

# Interpolation on common time base
num_samples = min(len(time_stamps_uav), len(time_stamps_wamv))
time_stamps_common = np.linspace(min(time_stamps_uav[0], time_stamps_wamv[0]),
                                 max(time_stamps_uav[-1], time_stamps_wamv[-1]),
                                 num_samples)

interp_uav_x = interp1d(time_stamps_uav, uav0_x, kind='linear', fill_value="extrapolate")
interp_uav_y = interp1d(time_stamps_uav, uav0_y, kind='linear', fill_value="extrapolate")
interp_wamv_x = interp1d(time_stamps_wamv, wamv_x, kind='linear', fill_value="extrapolate")
interp_wamv_y = interp1d(time_stamps_wamv, wamv_y, kind='linear', fill_value="extrapolate")

uav0_x_interp = interp_uav_x(time_stamps_common)
uav0_y_interp = interp_uav_y(time_stamps_common)
wamv_x_interp = interp_wamv_x(time_stamps_common)
wamv_y_interp = interp_wamv_y(time_stamps_common)

# Compute distances
distances = compute_distance(uav0_x_interp, uav0_y_interp, wamv_x_interp, wamv_y_interp)

# 過濾無效資料
valid_mask = ~np.isnan(distances) & ~np.isinf(distances)
distances = distances[valid_mask]
time_stamps_common = time_stamps_common[valid_mask]

# 可選：排除前100秒資料
if exclude_first_100s:
    time_threshold = time_stamps_common[0] + 100
    mask_100s = time_stamps_common >= time_threshold
    distances = distances[mask_100s]
    time_stamps_common = time_stamps_common[mask_100s]

# 誤差與統計
distance_error = distances - ideal_distance
mae = np.mean(np.abs(distance_error))
rmse = np.sqrt(np.mean(distance_error**2))

# 結果輸出
print("=== UAV 與 WAMV 距離誤差分析 ===")
print(f"理想距離: {ideal_distance} 公尺")
print(f"排除前 100 秒: {'是' if exclude_first_100s else '否'}")
print(f"平均誤差 (MAE): {mae:.4f} m")
print(f"均方根誤差 (RMSE): {rmse:.4f} m")