import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math

# === 資料初始化 ===
bag_file = "q_bpng2.bag"
time_stamps = []
phi_deg_list = []

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=["/uav0/base_pose_ground_truth"]):
        # 擷取 xy 平面速度分量
        v = msg.twist.twist.linear
        vx, vy = v.y, v.x
        norm_xy = math.hypot(vx, vy)

        # 忽略靜止或零速度點
        if norm_xy < 1e-6:
            continue

        # 目標向量 (1, 0)，只需取 vx/norm 即為 cos(phi)
        cos_phi = np.clip(vx / norm_xy, -1.0, 1.0)
        phi_rad = math.acos(cos_phi)
        if vy < 0:
            phi_rad = -phi_rad  # 使用 atan2-like 的方向性
        phi_deg = math.degrees(phi_rad)

        # 儲存
        phi_deg_list.append(phi_deg)
        time_stamps.append(t.to_sec())

# 時間標準化
t0 = time_stamps[0]
relative_time = [t - t0 for t in time_stamps]

# === 繪圖 ===
plt.figure(figsize=(10, 5))
plt.plot(relative_time, phi_deg_list, label='Psi angle', color='teal')
plt.axhline(y=30, color='red', linestyle='--', label='Desired psi')
plt.xlabel("Time (s)")
plt.ylabel("Signed Angle (degrees)")
plt.title("UAV0 Velocity Direction in XY Plane Over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()