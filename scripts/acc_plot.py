#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

# === 可設定繪圖起始與結束時間（以 rosbag 開始時間為基準）===
plot_start_time = 0.0  # 單位：秒
plot_end_time   = 180.0  # 單位：秒
dt = 0.1  # 時間取樣間隔 (秒)

# === 讀取 rosbag ===
bag = rosbag.Bag('output.bag')

# 紀錄 rosbag 絕對時間起點
bag_start_time = None

odom_times, odom_vels = [], []
cmd_times, cmd_accs = [], []

for topic, msg, t in bag.read_messages(topics=['/uav0/base_pose_ground_truth', '/uav0/bpn_cmd']):
    if bag_start_time is None:
        bag_start_time = t.to_sec()

    rel_time = t.to_sec() - bag_start_time

    if topic == '/uav0/base_pose_ground_truth':
        if plot_start_time <= rel_time <= plot_end_time:
            v = msg.twist.twist.linear
            odom_times.append(rel_time)
            odom_vels.append([v.x, v.y, v.z])

    elif topic == '/uav0/bpn_cmd':
        if plot_start_time <= rel_time <= plot_end_time:
            cmd_times.append(rel_time)
            cmd_accs.append([msg.x, msg.y, msg.z])

bag.close()

odom_times = np.array(odom_times)
odom_vels = np.array(odom_vels)
cmd_times = np.array(cmd_times)
cmd_accs = np.array(cmd_accs)

# === 下採樣與速度差分估計加速度 ===
sample_times = np.arange(plot_start_time, plot_end_time, dt)
interp_v = np.array([np.interp(sample_times, odom_times, odom_vels[:, i]) for i in range(3)]).T
accels = np.diff(interp_v, axis=0) / dt
accel_times = sample_times[1:]

# === 指令加速度插值 ===
interp_cmd = np.array([np.interp(accel_times, cmd_times, cmd_accs[:, i]) for i in range(3)]).T

# === 繪圖 ===
axes = ['x', 'y', 'z']
plt.figure(figsize=(12, 8))
for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.plot(accel_times, accels[:, i], label='Estimated Accel ({})'.format(axes[i]), color='b')
    plt.plot(accel_times, interp_cmd[:, i], label='Command Accel ({})'.format(axes[i]), color='r', linestyle='--')
    plt.ylabel('Accel (m/s²)')
    plt.legend()
    plt.grid(True)

plt.xlabel('Time (s)')
plt.suptitle('UAV Acceleration vs Command (From {:.1f}s to {:.1f}s)'.format(plot_start_time, plot_end_time))
plt.tight_layout()
plt.show()

