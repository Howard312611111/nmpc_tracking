import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Function to compute Euclidean distance in the XY plane
def compute_distance(uav_x, uav_y, wamv_x, wamv_y):
    return np.sqrt((np.array(uav_x) - np.array(wamv_x))**2 + (np.array(uav_y) - np.array(wamv_y))**2)

# Load the bag file
bag_file = "Rework_judg2.bag"
bag = rosbag.Bag(bag_file, "r")

# Lists to store data
time_stamps_uav, uav0_x, uav0_y = [], [], []
time_stamps_wamv, wamv_x, wamv_y = [], [], []

# Read messages from the bag
for topic, msg, t in bag.read_messages(topics=["/uav0/base_pose_ground_truth", "/wamv/base_pose_ground_truth"]):
    if topic == "/uav0/base_pose_ground_truth":
        uav0_x.append(msg.pose.pose.position.x)
        uav0_y.append(msg.pose.pose.position.y)
        time_stamps_uav.append(t.to_sec())
    elif topic == "/wamv/base_pose_ground_truth":
        wamv_x.append(msg.pose.pose.position.x)
        wamv_y.append(msg.pose.pose.position.y)
        time_stamps_wamv.append(t.to_sec())

bag.close()

# Normalize the number of messages using interpolation
num_samples = min(len(time_stamps_uav), len(time_stamps_wamv))
time_stamps_common = np.linspace(min(time_stamps_uav[0], time_stamps_wamv[0]), 
                                 max(time_stamps_uav[-1], time_stamps_wamv[-1]), num_samples)

interp_uav_x = interp1d(time_stamps_uav, uav0_x, kind='linear', fill_value="extrapolate")
interp_uav_y = interp1d(time_stamps_uav, uav0_y, kind='linear', fill_value="extrapolate")
interp_wamv_x = interp1d(time_stamps_wamv, wamv_x, kind='linear', fill_value="extrapolate")
interp_wamv_y = interp1d(time_stamps_wamv, wamv_y, kind='linear', fill_value="extrapolate")

uav0_x_interp = interp_uav_x(time_stamps_common)
uav0_y_interp = interp_uav_y(time_stamps_common)
wamv_x_interp = interp_wamv_x(time_stamps_common)
wamv_y_interp = interp_wamv_y(time_stamps_common)

# Compute distance
distances = compute_distance(uav0_x_interp, uav0_y_interp, wamv_x_interp, wamv_y_interp)

# Plot results with time stamps on x-axis and distance on y-axis
fig, ax = plt.subplots()
ax.plot(time_stamps_common, distances, color="blue", label="Distance in XY Plane")
ax.axhline(y=540, color='red', linestyle='--', label="Target Distance (540m)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Distance (m)")
ax.set_title("Distance Between UAV and Target in XY Plane")
ax.legend()
plt.show()

print(f"Mean Distance in XY Plane: {np.mean(distances):.4f} m")