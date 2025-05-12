import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Load the bag file
bag_file = "Rework_judg2.bag"
bag = rosbag.Bag(bag_file, "r")

# Lists to store data
time_stamps_wamv, wamv_x = [], []
time_stamps_ukf, ukf_x = [], []

# Read messages from the bag
for topic, msg, t in bag.read_messages(topics=["/wamv/base_pose_ground_truth", "/uav0/estimation/ukf/output_data"]):
    if topic == "/wamv/base_pose_ground_truth":
        wamv_x.append(msg.twist.twist.linear.y)
        time_stamps_wamv.append(t.to_sec())
    elif topic == "/uav0/estimation/ukf/output_data":
        ukf_x.append(msg.target_vel.y)
        time_stamps_ukf.append(t.to_sec())

bag.close()

# Ensure we have data
if len(time_stamps_wamv) == 0 or len(time_stamps_ukf) == 0:
    print("Error: Missing data from one or both topics!")
    exit()

# Normalize the number of messages using interpolation
num_samples = min(len(time_stamps_wamv), len(time_stamps_ukf))
time_stamps_common = np.linspace(min(time_stamps_wamv[0], time_stamps_ukf[0]), 
                                 max(time_stamps_wamv[-1], time_stamps_ukf[-1]), num_samples)

# Perform interpolation safely
try:
    interp_wamv_x = interp1d(time_stamps_wamv, wamv_x, kind='linear', fill_value="extrapolate", bounds_error=False)
    interp_ukf_x = interp1d(time_stamps_ukf, ukf_x, kind='linear', fill_value="extrapolate", bounds_error=False)

    wamv_x_interp = interp_wamv_x(time_stamps_common)
    ukf_x_interp = interp_ukf_x(time_stamps_common)

    # Ensure no NaN or Inf values
    valid_mask = ~np.isnan(wamv_x_interp) & ~np.isnan(ukf_x_interp) & ~np.isinf(wamv_x_interp) & ~np.isinf(ukf_x_interp)
    
    # Trim invalid values
    time_stamps_common = time_stamps_common[valid_mask]
    wamv_x_interp = wamv_x_interp[valid_mask]
    ukf_x_interp = ukf_x_interp[valid_mask]

except Exception as e:
    print(f"Interpolation error: {e}")
    exit()

# Plot results with time stamps on x-axis and position on y-axis
fig, ax = plt.subplots()
ax.plot(time_stamps_common, wamv_x_interp, color="blue", label="Target Velocity Y")
ax.plot(time_stamps_common, ukf_x_interp, color="red", linestyle="-.", label="Esitmated Velocity Y")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (m/s)")
ax.set_title("Target Velocity vs Estimated Velocity Over Time")
ax.legend()
plt.show()
