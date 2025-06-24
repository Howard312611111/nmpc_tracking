import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 替換為你的 bag 檔路徑
bag_path = 'output.bag'

# 儲存 UAV0 和 WAMV 的座標
uav_positions = []
wamv_positions = []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == '/uav0/base_pose_ground_truth':
            pos = msg.pose.pose.position
            uav_positions.append([pos.x, pos.y, pos.z])
        elif topic == '/fake_odometry':
            pos = msg.pose.pose.position
            wamv_positions.append([pos.x, pos.y, pos.z])

# 轉為 numpy 陣列
uav_positions = np.array(uav_positions)
wamv_positions = np.array(wamv_positions)

# 繪製 3D 圖
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

if uav_positions.size > 0:
    ax.plot(uav_positions[:,0], uav_positions[:,1], uav_positions[:,2], label='UAV0', color='blue')
if wamv_positions.size > 0:
    ax.plot(wamv_positions[:,0], wamv_positions[:,1], wamv_positions[:,2], label='WAMV', color='green')

ax.set_title('3D Trajectory of UAV0 and WAMV')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()