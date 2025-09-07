import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file (update the path)
csv_path = "/home/shekoufeh/FIF_ws/src/rpg_information_field/act_map/FOVData/optimizer_accuracy_file.csv"

df = pd.read_csv(csv_path)


# Extract columns
degree_diff = df.iloc[:, 0]  # Degree difference
landmarks_opt = df.iloc[:, 1]  # Landmarks in optimized view
landmarks_bf = df.iloc[:, 2]  # Landmarks in brute force view
poses = np.arange(len(degree_diff))  # Camera pose indices

# Compute accuracy percentage
accuracy = (landmarks_bf / landmarks_opt) * 100
accuracy[np.isnan(accuracy)] = 0  # Handle division by zero cases
mean_accuracy = np.mean(accuracy)  # Compute mean accuracy

# --- PLOTS ---
plt.figure(figsize=(16, 8))

# (1) Line plot: Degree difference over poses
plt.subplot(2, 2, 1)
plt.plot(poses, degree_diff, marker='o', linestyle='-', color='r', label="Degree Difference")
plt.xlabel("Camera Pose")
plt.ylabel("Degree Difference (°)")
plt.title("Degree Difference per Camera Pose")
plt.grid(True)
plt.legend()

# (2) Bar plot: Landmarks comparison per pose
plt.subplot(2, 2, 2)
bar_width = 0.4
plt.bar(poses - bar_width/2, landmarks_opt, width=bar_width, color='orange', label="Brute Force View")
plt.bar(poses + bar_width/2, landmarks_bf, width=bar_width, color='blue', label="Optimized View")
plt.xlabel("Camera Pose")
plt.ylabel("Number of Landmarks")
plt.title("Landmarks Seen per Approach")
plt.legend()
plt.grid(axis='y')

# (3) Histogram: Distribution of Degree Differences
plt.subplot(2, 2, 3)
plt.hist(degree_diff, bins=15, color='g', alpha=0.7, edgecolor='black')
plt.xlabel("Degree Difference (°)")
plt.ylabel("Frequency")
plt.title("Distribution of Degree Differences")
plt.grid(True)

# (4) Accuracy Curve: Brute Force vs. Optimized Landmarks
plt.subplot(2, 2, 4)
plt.plot(poses, accuracy, marker='s', linestyle='-', color='purple', label="BF Accuracy (%)")
plt.axhline(y=100, color='gray', linestyle='--', label="100% Reference")
plt.axhline(y=mean_accuracy, color='red', linestyle='--', label=f"Mean Accuracy: {mean_accuracy:.2f}%")
plt.xlabel("Camera Pose")
plt.ylabel("Accuracy (%)")
plt.title("Brute Force View Accuracy Over Poses")
plt.ylim(0, 110)
plt.grid(True)
plt.legend()

# Show plots
plt.tight_layout()
plt.show()
