import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Function to compute RMSE safely
def compute_rmse(actual, estimated):
    actual = np.array(actual)
    estimated = np.array(estimated)
    
    # Remove NaN and Inf values before computing RMSE
    mask = ~np.isnan(actual) & ~np.isnan(estimated) & ~np.isinf(actual) & ~np.isinf(estimated)
    if np.sum(mask) == 0:  # Check if there's valid data left
        return float('nan')  
    
    return np.sqrt(np.mean((actual[mask] - estimated[mask])**2))

# Load the bag file
bag_file = "Rework_judg2.bag"
bag = rosbag.Bag(bag_file, "r")

# Lists to store data
time_stamps_wamv, wamv_x = [], []
time_stamps_ukf, ukf_x = [], []

# Read messages from the bag
for topic, msg, t in bag.read_messages(topics=["/wamv/base_pose_ground_truth", "/uav0/estimation/ukf/output_data"]):
    if topic == "/wamv/base_pose_ground_truth":
        wamv_x.append(msg.twist.twist.linear.x)
        time_stamps_wamv.append(t.to_sec())
    elif topic == "/uav0/estimation/ukf/output_data":
        ukf_x.append(msg.target_vel.x)
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

# Compute error
errors = wamv_x_interp - ukf_x_interp
rmse = compute_rmse(wamv_x_interp, ukf_x_interp)

# Plot results with time stamps on x-axis and error on y-axis
fig, ax = plt.subplots()
ax.plot(time_stamps_common, errors, color="blue", label="Error of X Velocity")
ax.axhline(y=rmse, color='red', linestyle='--', label=f"RMSE ({rmse:.4f} m/s)")
# ax.axhline(y=-rmse, color='red', linestyle='--')  # Add negative RMSE line
ax.set_xlabel("Time (s)")
ax.set_ylabel("Error (m/s)")
ax.set_title("Error Between Target's X Velocity and Esitmated X Velocity")
ax.legend()
plt.show()

print(f"Overall RMSE in X Position: {rmse:.4f} m")
