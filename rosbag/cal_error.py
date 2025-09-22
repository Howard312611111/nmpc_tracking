import rosbag
import numpy as np
from scipy.interpolate import interp1d

# === CONFIG: 是否排除前100秒資料 ===
exclude_first_100s = 1  # 設為 False 則會計算全部時間

# 讀取 bag 檔案
bag_file = "gndframe.bag"
bag = rosbag.Bag(bag_file, "r")

# Ground truth 與 UKF 的速度資料
time_stamps_wamv, wamv_vx, wamv_vy = [], [], []
time_stamps_ukf, ukf_vx, ukf_vy = [], [], []

# 從 bag 中讀取資料
for topic, msg, t in bag.read_messages(topics=["/wamv/base_pose_ground_truth", "/uav0/estimation/ukf/output_data"]):
    if topic == "/wamv/base_pose_ground_truth":
        wamv_vx.append(msg.pose.pose.position.x)
        wamv_vy.append(msg.pose.pose.position.y)
        time_stamps_wamv.append(t.to_sec())
    elif topic == "/uav0/estimation/ukf/output_data":
        ukf_vx.append(msg.target_pose.x)
        ukf_vy.append(msg.target_pose.y)
        time_stamps_ukf.append(t.to_sec())

bag.close()

# 內插資料對齊時間軸
num_samples = min(len(time_stamps_wamv), len(time_stamps_ukf))
time_stamps_common = np.linspace(min(time_stamps_wamv[0], time_stamps_ukf[0]),
                                 max(time_stamps_wamv[-1], time_stamps_ukf[-1]),
                                 num_samples)

interp_wamv_vx = interp1d(time_stamps_wamv, wamv_vx, kind='linear', fill_value="extrapolate", bounds_error=False)
interp_wamv_vy = interp1d(time_stamps_wamv, wamv_vy, kind='linear', fill_value="extrapolate", bounds_error=False)
interp_ukf_vx = interp1d(time_stamps_ukf, ukf_vx, kind='linear', fill_value="extrapolate", bounds_error=False)
interp_ukf_vy = interp1d(time_stamps_ukf, ukf_vy, kind='linear', fill_value="extrapolate", bounds_error=False)

wamv_vx_i = interp_wamv_vx(time_stamps_common)
wamv_vy_i = interp_wamv_vy(time_stamps_common)
ukf_vx_i = interp_ukf_vx(time_stamps_common)
ukf_vy_i = interp_ukf_vy(time_stamps_common)

# 計算絕對速度
wamv_speed = np.sqrt(np.square(wamv_vx_i) + np.square(wamv_vy_i))
ukf_speed = np.sqrt(np.square(ukf_vx_i) + np.square(ukf_vy_i))

# 移除 NaN 與 inf 資料
valid_mask = ~np.isnan(wamv_speed) & ~np.isnan(ukf_speed) & ~np.isinf(wamv_speed) & ~np.isinf(ukf_speed)
wamv_speed = wamv_speed[valid_mask]
ukf_speed = ukf_speed[valid_mask]
time_stamps_common = time_stamps_common[valid_mask]

# ✅ 選擇是否排除前100秒資料
if exclude_first_100s:
    start_time = time_stamps_common[0] + 100.0
    mask_100s = time_stamps_common >= start_time
    wamv_speed = wamv_speed[mask_100s]
    ukf_speed = ukf_speed[mask_100s]
    time_stamps_common = time_stamps_common[mask_100s]

# 計算誤差
mae = np.mean(np.abs(wamv_speed - ukf_speed))
rmse = np.sqrt(np.mean(np.square(wamv_speed - ukf_speed)))

# 輸出結果
print("=== 絕對位置誤差結果 ===")
print(f"排除前100秒: {'是' if exclude_first_100s else '否'}")
print(f"平均誤差 (MAE): {mae:.4f} m/s")
print(f"均方根誤差 (RMSE): {rmse:.4f} m/s")