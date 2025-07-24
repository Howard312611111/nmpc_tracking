import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math

# === 固定參數 ===
theta_deg = -30
phi_deg = 30
theta = np.radians(theta_deg)
phi = np.radians(phi_deg)

# 計算 u_f_ 向量（單位向量）
u_f = np.array([
    np.cos(theta) * np.cos(phi),
    np.cos(theta) * np.sin(phi),
    -np.sin(theta)
])
u_f_norm = u_f / np.linalg.norm(u_f)

# === 資料載入 ===
bag_file = "output.bag"
time_stamps = []
errors_deg = []

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=["/uav0/base_pose_ground_truth"]):
        # 取出線速度向量
        v = msg.twist.twist.linear
        v_m = np.array([v.x, -v.y, -v.z])
        if np.linalg.norm(v_m) == 0:
            continue  # 跳過零速樣本，避免除以0
        v_m_norm = v_m / np.linalg.norm(v_m)

        # 計算夾角誤差（rad -> deg）
        dot_product = np.clip(np.dot(v_m_norm, u_f_norm), -1.0, 1.0)
        error_rad = math.acos(dot_product)
        error_deg = np.degrees(error_rad)

        errors_deg.append(error_deg)
        time_stamps.append(t.to_sec())

# 時間轉為相對時間
t0 = time_stamps[0]
relative_time = [t - t0 for t in time_stamps]

# === 繪圖 ===
plt.figure(figsize=(10, 5))
plt.plot(relative_time, errors_deg, label='Angle Error (°)', color='purple')
plt.xlabel("Time (s)")
plt.ylabel("Angular Error (degrees)")
plt.title("Angle Between UAV0 Velocity and u_f Over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()