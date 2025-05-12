import rosbag
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def extract_data(bag, topics):
    GT_poses = []
    for topic, msg, t in bag.read_messages(topics):
        GT_poses.append(msg.pose.pose)
    return GT_poses

def plotFromBag(bag, name):
    pose1 = extract_data(bag, '/uav0/base_pose_ground_truth')
    pose2 = extract_data(bag, '/wamv/base_pose_ground_truth')

    plot_combined_position_2D_animated(pose1, pose2)

def plot_combined_position_2D_animated(GT_poses1, est_poses1):
    # Extract positions for Ground Truth
    GT_x1 = [pose.position.x for pose in GT_poses1]
    GT_y1 = [pose.position.z for pose in GT_poses1]

    # Extract positions for Estimated poses
    est_x1 = [pose.position.x for pose in est_poses1]
    est_y1 = [pose.position.z for pose in est_poses1]

    fig, ax = plt.subplots()
    ax.set_xlim(min(GT_x1 + est_x1) - 1, max(GT_x1 + est_x1) + 1)
    ax.set_ylim(-100, 600)  # Adjusted y-axis limits as in the original code
    ax.set_xlabel('X-axis (m)')
    ax.set_ylabel('Z-axis (m)')
    ax.set_title('xz plane trajectory')
    
    line_gt, = ax.plot([], [], label='UAV', lw=2)
    line_est, = ax.plot([], [], label='Target', lw=2)
    ax.legend()

    def init():
        line_gt.set_data([], [])
        line_est.set_data([], [])
        return line_gt, line_est

    def update(frame):
        # Only use every 10th point for the animation
        step = 10
        line_gt.set_data(GT_x1[:frame * step], GT_y1[:frame * step])
        line_est.set_data(est_x1[:int(frame * step * 1.3)], est_y1[:int(frame * step * 1.3)])
        return line_gt, line_est

    # Set the number of frames to be len(GT_x1) // 10 to show every 10th point
    ani = FuncAnimation(fig, update, frames=len(GT_x1) // 10, init_func=init, blit=True, interval=5)

    # Save animation as a GIF
    ani.save('trajectory_animation.gif', writer='pillow', fps=60, dpi=80)
    #print("Animation saved as 'trajectory_animation.gif'")

    plt.show()

# Path to your bag file
file1 = 'all_in.bag'
bag1 = rosbag.Bag(file1)

# Plot from the bag file
plotFromBag(bag1, 'THEIF, Only one neighbor robot has absolute position rate 5hz')

