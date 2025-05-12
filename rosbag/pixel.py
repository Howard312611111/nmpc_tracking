import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Load the bag file
bag_file = "Rework_judg2.bag"
bag = rosbag.Bag(bag_file, "r")

# Lists to store data
time_stamps, errors = [], []

# Read messages from the bag
for topic, msg, t in bag.read_messages(topics=["/uav0/estimation/ukf/camera_features"]):
    u = msg.u.data
    cu = msg.cu.data
    error = u - cu
    time_stamps.append(t.to_sec())
    errors.append(error)
print(cu)
bag.close()

# Ensure we have data
if len(time_stamps) == 0:
    print("Error: No data found in topic!")
    exit()

# Convert lists to numpy arrays
time_stamps = np.array(time_stamps)
errors = np.array(errors)

# Plot results as scatter plot
plt.figure(figsize=(10, 5))
plt.scatter(time_stamps, errors, color="blue", s=10, label="Error")
plt.axhline(y=640, color="red", linestyle="--", label="Image border")
plt.axhline(y=-640, color="red", linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Error (pixel)")
plt.title("u-Direction")
plt.ylim([-700, 700])  # Fix y-axis range
plt.legend()
plt.grid()
plt.show()