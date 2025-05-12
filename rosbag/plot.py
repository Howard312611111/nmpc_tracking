import rosbag
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def extract_data(bag, topics):
    timestamps = []

    GT_poses = []
    for topic, msg, t in bag.read_messages(topics):
    #    print('HIHI')
    # Extract relevant data from the message
        GT_poses.append(msg.pose.pose)
    return GT_poses

def plotFromBag(bag, name):
    pose1 = extract_data(bag, '/uav0/base_pose_ground_truth')
    pose2 = extract_data(bag, '/wamv/base_pose_ground_truth')

    plot_combined_position_3D(pose1, pose2)

def plot_combined_position_3D(GT_poses1, est_poses1):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract positions for Ground Truth
    GT_x1 = [pose.position.x for pose in GT_poses1]
    GT_y1 = [pose.position.y for pose in GT_poses1]
    GT_z1 = [pose.position.z for pose in GT_poses1]

    # Extract positions for Estimated poses
    est_x1 = [pose.position.x for pose in est_poses1]
    est_y1 = [pose.position.y for pose in est_poses1]
    est_z1 = [pose.position.z for pose in est_poses1]

    # Plot Ground Truth
    ax.plot(GT_x1, GT_y1, GT_z1, label='UAV')

    # Plot Estimated Poses
    ax.plot(est_x1, est_y1, est_z1, label='Target')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()

    plt.title('Combined 3D Poses')
    plt.show()

# Path to your bag file
file1 = 'Rework_judg.bag'
bag1 = rosbag.Bag(file1)

# Plot from the bag file
plotFromBag(bag1, 'THEIF, Only one neighbor robot has absolute position rate 5hz')
