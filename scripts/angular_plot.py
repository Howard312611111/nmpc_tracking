#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_matrix
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry

# === 設定繪圖參數 ===
plot_start_time = 0.0  # s
plot_end_time = 60.0    # s
dt = 0.1

# === 初始化 rosbag ===
bag = rosbag.Bag('nmpc_output.bag')

bag_start_time = None
odom_times, ang_vels_body = [], []
cmd_times, body_rates = [], []

# === 從 rosbag 讀取與轉換 ===
for topic, msg, t in bag.read_messages(topics=[
    '/uav0/base_pose_ground_truth',
    '/uav0/mavros/setpoint_raw/attitude'
]):
    if bag_start_time is None:
        bag_start_time = t.to_sec()
    rel_time = t.to_sec() - bag_start_time

    # --- 實際角速度（從 world frame 轉到 body frame）---
    if topic == '/uav0/base_pose_ground_truth' and plot_start_time <= rel_time <= plot_end_time:
        # quaternion 轉 rotation matrix
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        R = quaternion_matrix(quat)[:3, :3]  # rotation from body to world

        # angular velocity in world frame
        w_world = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # transform to body frame
        w_body = R.T.dot(w_world)
        ang_vels_body.append(w_body)
        odom_times.append(rel_time)

    # --- 控制器指令的角速度 ---
    elif topic == '/uav0/mavros/setpoint_raw/attitude' and plot_start_time <= rel_time <= plot_end_time:
        br = msg.body_rate
        body_rates.append([br.x, br.y, -br.z])
        cmd_times.append(rel_time)

bag.close()

# === 資料插值與繪圖 ===
odom_times = np.array(odom_times)
ang_vels_body = np.array(ang_vels_body)
cmd_times = np.array(cmd_times)
body_rates = np.array(body_rates)

sample_times = np.arange(plot_start_time, plot_end_time, dt)

interp_ang = np.array([
    np.interp(sample_times, odom_times, ang_vels_body[:, i]) for i in range(3)
]).T

interp_cmd = np.array([
    np.interp(sample_times, cmd_times, body_rates[:, i]) for i in range(3)
]).T

# === 繪圖 ===
axes = ['x', 'y', 'z']
plt.figure(figsize=(12, 8))
for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.plot(sample_times, interp_ang[:, i], label='Measured BodyRate ({})'.format(axes[i]), color='b')
    plt.plot(sample_times, interp_cmd[:, i], label='Command BodyRate ({})'.format(axes[i]), color='r', linestyle='--')
    plt.ylabel('Angular Rate (rad/s)')
    plt.legend()
    plt.grid(True)

plt.xlabel('Time (s)')
plt.suptitle('UAV Angular Velocity (Converted to Body Frame) vs Command ({:.1f}s - {:.1f}s)'.format(
    plot_start_time, plot_end_time))
plt.tight_layout()
plt.show()
