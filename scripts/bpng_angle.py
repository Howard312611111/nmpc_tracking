import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tf_trans

# 替換為你的 bag 檔案路徑
bag_file = "output.bag"

# 儲存資料
time_stamps = []
pitch_angles = []

# 讀取 rosbag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav0/base_pose_ground_truth']):
        # 取得四元數
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        # 轉換為 Euler 角
        roll, pitch, yaw = tf_trans.euler_from_quaternion(quat)
        pitch_deg = np.degrees(pitch)

        # 存儲時間與仰角
        time_stamps.append(t.to_sec())
        pitch_angles.append(pitch_deg)

# 計算相對時間
start_time = time_stamps[0]
relative_time = [t - start_time for t in time_stamps]

# 繪圖
plt.figure(figsize=(10, 5))
plt.plot(relative_time, pitch_angles, label='Pitch Angle θ (degrees)', color='orange')
plt.axhline(y=30, color='red', linestyle='--', linewidth=2, label='θ = 30°')  # 紅線標記
plt.xlabel('Time (s)')
plt.ylabel('Pitch (θ) in Degrees')
plt.title('UAV0 Pitch Angle Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()