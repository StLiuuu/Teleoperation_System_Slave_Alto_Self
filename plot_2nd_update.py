import csv
import numpy as np
import matplotlib.pyplot as plt


data_obs = []
data_real = []
data_pred = []

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


obs = np.array(data_obs) if len(data_obs) > 0 else np.zeros((0, 3))
real = np.array(data_real) if len(data_real) > 0 else np.zeros((0, 3))
pred = np.array(data_pred) if len(data_pred) > 0 else np.zeros((0, 3))


fig, axs = plt.subplots(3, 1, figsize=(10, 18))


axs[0].plot(obs[:, 1], obs[:, 2], 'o-', label="Initial Trajectory (OBS)", color="blue", alpha=0.6)
axs[0].plot(real[:, 1], real[:, 2], 'o-', label="Robot Trajectory (REAL)", color="green", alpha=0.6)
axs[0].plot(pred[:, 1], pred[:, 2], 'o-', label="Predicted Trajectory (PRED)", color="orange", alpha=0.6)



axs[0].set_xlabel("X")
axs[0].set_ylabel("Y")
axs[0].set_title("Spatial Trajectories")
axs[0].legend()
axs[0].grid(True)


axs[1].plot(obs[:, 0], obs[:, 1], 'o-', label="Initial Trajectory X", color="blue")
axs[1].plot(real[:, 0], real[:, 1], 'o-', label="Robot Trajectory X", color="green")
axs[1].plot(pred[:, 0], pred[:, 1], 'o-', label="Predicted Trajectory X", color="orange")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("X Position")
axs[1].set_title("X Position over Time")
axs[1].legend()
axs[1].grid(True)


axs[2].plot(obs[:, 0], obs[:, 2], 'o-', label="Initial Trajectory Y", color="blue")
axs[2].plot(real[:, 0], real[:, 2], 'o-', label="Robot Trajectory Y", color="green")
axs[2].plot(pred[:, 0], pred[:, 2], 'o-', label="Predicted Trajectory Y", color="orange")
axs[2].set_xlabel("Time (s)")
axs[2].set_ylabel("Y Position")
axs[2].set_title("Y Position over Time")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()