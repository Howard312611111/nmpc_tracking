import rosbag
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

def extract_data(bag, topics):
    GT_poses = []
    for topic, msg, t in bag.read_messages(topics):
        GT_poses.append(msg.pose.pose)
    return GT_poses

def plotFromBag(bag, name):
    pose1 = extract_data(bag, '/uav0/base_pose_ground_truth')
    pose2 = extract_data(bag, '/wamv/base_pose_ground_truth')

    plot_combined_position_2D_static(pose1, pose2)

def plot_combined_position_2D_static(GT_poses1, est_poses1):
    # Extract positions for Ground Truth
    GT_x1 = [pose.position.x for pose in GT_poses1]
    GT_y1 = [pose.position.y for pose in GT_poses1]

    # Extract positions for Estimated poses
    est_x1 = [pose.position.x for pose in est_poses1]
    est_y1 = [pose.position.y for pose in est_poses1]

    fig, ax = plt.subplots()
    ax.set_xlim(min(GT_x1 + est_x1) - 1, max(GT_x1 + est_x1) + 1)
    ax.set_ylim(-2500, 2500)  # Adjusted y-axis limits
    ax.set_xlabel('X-axis (m)')
    ax.set_ylabel('Y-axis (m)')
    ax.set_title('XY Plane Trajectory')

    # Plot static 2D trajectory
    ax.plot(GT_x1, GT_y1, label='UAV', lw=2)
    ax.plot(est_x1, est_y1, label='Target', lw=2)
    ax.legend()

    plt.show()

# Path to your bag file
file1 = 'Rework_judg2.bag'
bag1 = rosbag.Bag(file1)

# Plot from the bag file
plotFromBag(bag1, 'THEIF, Only one neighbor robot has absolute position rate 5hz')
