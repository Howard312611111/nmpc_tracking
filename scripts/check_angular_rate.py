#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

# === 設定參數 ===
bag_path = 'nmpc_output.bag'
plot_start_time = 0.0  # 相對時間 (秒)
plot_end_time = 45.0
dt = 0.1

# === 初始化容器 ===
bag = rosbag.Bag(bag_path)
bag_start_time = None

odom_times, uav_ang_body = [], []
nmpc_times, nmpc_cmds = [], []

# === 讀取 rosbag ===
for topic, msg, t in bag.read_messages(topics=[
    '/uav0/base_pose_ground_truth', '/nmpc_ans']):
    if bag_start_time is None:
        bag_start_time = t.to_sec()
    rel_time = t.to_sec() - bag_start_time

    if plot_start_time <= rel_time <= plot_end_time:
        if topic == '/uav0/base_pose_ground_truth':
            q = msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            R = quaternion_matrix(quat)[:3, :3]

            ang_world = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])

            ang_body = ang_world  # 轉到 body frame
            uav_ang_body.append([ang_body[0], ang_body[1]])  # 只取 roll, pitch
            odom_times.append(rel_time)

        elif topic == '/nmpc_ans':
            data = msg.data
            if len(data) >= 3:
                nmpc_cmds.append([data[1], data[2]])  # roll_dot, pitch_dot
                nmpc_times.append(rel_time)

bag.close()

# === 插值至共同時間軸 ===
odom_times = np.array(odom_times)
uav_ang_body = np.array(uav_ang_body)
nmpc_times = np.array(nmpc_times)
nmpc_cmds = np.array(nmpc_cmds)

sample_times = np.arange(plot_start_time, plot_end_time, dt)

interp_uav = np.array([
    np.interp(sample_times, odom_times, uav_ang_body[:, i]) for i in range(2)
]).T

interp_nmpc = np.array([
    np.interp(sample_times, nmpc_times, nmpc_cmds[:, i]) for i in range(2)
]).T

# === 繪圖 ===
axes = ['Roll Rate', 'Pitch Rate']
plt.figure(figsize=(12, 6))
for i in range(2):
    plt.subplot(2, 1, i + 1)
    plt.plot(sample_times, interp_uav[:, i], label='UAV Actual ({})'.format(axes[i]), color='b')
    plt.plot(sample_times, interp_nmpc[:, i], label='NMPC Command ({})'.format(axes[i]), color='r', linestyle='--')
    plt.ylabel('Rate (rad/s)')
    plt.grid(True)
    plt.legend()

plt.xlabel('Time (s)')
plt.suptitle('NMPC Roll/Pitch Rate Command vs UAV Actual (Body Frame)')
plt.tight_layout()
plt.show()
