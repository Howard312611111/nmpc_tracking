import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Load the bag file
bag_file = "Rework_judg.bag"
bag = rosbag.Bag(bag_file, "r")

# Lists to store data
time_stamps, positions = [], []

# Read messages from the bag (limit to 12000 messages)
message_count = 0

for topic, msg, t in bag.read_messages(topics=["/uav0/gimbal/joint_states"]):
    if "cgo3_camera_joint" in msg.name:
        index = msg.name.index("cgo3_camera_joint")
        position = msg.position[index]
        time_stamps.append(t.to_sec())
        positions.append(position)
        message_count += 1
        if message_count >= 12000:
            break

bag.close()

# Ensure we have data
if len(time_stamps) == 0:
    print("Error: No data found in topic!")
    exit()

# Convert lists to numpy arrays
time_stamps = np.array(time_stamps)
positions = np.array(positions)

# Plot results
plt.figure(figsize=(10, 5))
plt.plot(time_stamps, positions, color="blue", label="Gimbal Tilt Angle")
plt.axhline(y=0, color='red', linestyle='--', label='Tilt Angle Limit')  # Added line
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Camera Joint Position with Modified Constraint")
plt.legend()
plt.grid()
plt.show()