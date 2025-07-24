import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math

# === 資料初始化 ===
bag_file = "output.bag"
time_stamps = []
signed_phi_deg_list = []

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=["/uav0/base_pose_ground_truth"]):
        # 取得 UAV 速度向量
        v = msg.twist.twist.linear
        vx, vy, vz = v.x, v.y, v.z
        norm = math.sqrt(vx**2 + vy**2 + vz**2)

        if norm < 1e-6:
            continue

        # 計算夾角：速度向量與 XY 平面的夾角
        theta_rad = math.asin(np.clip(vz / norm, -1.0, 1.0))  # 自動帶正負
        theta_deg = math.degrees(theta_rad)

        signed_phi_deg_list.append(theta_deg)

        time_stamps.append(t.to_sec())

# 相對時間軸
t0 = time_stamps[0]
relative_time = [t - t0 for t in time_stamps]

# === 繪圖 ===
plt.figure(figsize=(10, 5))
plt.plot(relative_time, signed_phi_deg_list, label='Theta angle', color='blue')
plt.axhline(y=-30, color='red', linestyle='--', label='Desired theta')
plt.xlabel("Time (s)")
plt.ylabel("Signed Angle (degrees)")
plt.title("UAV0 Velocity Inclination vs XY Plane Over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()