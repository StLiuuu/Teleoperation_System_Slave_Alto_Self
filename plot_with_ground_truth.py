import csv
import numpy as np
import matplotlib.pyplot as plt

# Initialize data containers
data_obs = []
data_real = []
data_pred = []
data_ground = []

# Read the CSV file
csv_filename = "recorded_trajectory_data.csv"
with open(csv_filename, "r") as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        data_type = row[0]
        t = float(row[1])
        x = float(row[2])
        y = float(row[3])
        if data_type == "OBS":
            data_obs.append((t, x, y))
        elif data_type == "REAL":
            data_real.append((t, x, y))
        elif data_type == "PRED":
            data_pred.append((t, x, y))
        # elif data_type == "GROUND":
        #     data_ground.append((t, x, y))

# Read the CSV file
csv_filename = "ground_Truth.csv"
with open(csv_filename, "r") as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        data_type = row[0]
        t = float(row[1])
        x = float(row[2])
        y = float(row[3])
        if data_type == "GROUND":
            data_ground.append((t, x, y))

# Convert to NumPy arrays or empty arrays if missing
obs = np.array(data_obs) if data_obs else np.zeros((0, 3))
real = np.array(data_real) if data_real else np.zeros((0, 3))
pred = np.array(data_pred) if data_pred else np.zeros((0, 3))
ground = np.array(data_ground) if data_ground else np.zeros((0, 3))

# Create subplots
fig, axs = plt.subplots(3, 1, figsize=(10, 18))

# Trajectory in space (X vs Y)
if obs.size: axs[0].plot(obs[:, 1], obs[:, 2], 'o-', label="Initial Trajectory (OBS)", color="blue", alpha=0.6)
if real.size: axs[0].plot(real[:, 1], real[:, 2], 'o-', label="Robot Trajectory (REAL)", color="green", alpha=0.6)
if pred.size: axs[0].plot(pred[:, 1], pred[:, 2], 'o-', label="Predicted Trajectory (PRED)", color="orange", alpha=0.6)
if ground.size: axs[0].plot(ground[:, 1], ground[:, 2], '--', label="Ground Truth (GROUND)", color="black", alpha=0.8)

axs[0].set_xlabel("X")
axs[0].set_ylabel("Y")
axs[0].set_title("Spatial Trajectories")
axs[0].legend()
axs[0].grid(True)




plt.tight_layout()
plt.show()
