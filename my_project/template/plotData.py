import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV data
data = pd.read_csv("robot_position.csv")

# Plot the 2D trajectory (x-y plane)
plt.figure(figsize=(8,6))
plt.plot(data['time'], data['x'], marker='o', linestyle='-')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Trunk Trajectory (X-Y Plane)')
plt.grid(True)
plt.axis('equal')
plt.show()

# Optionally, plot the time histories of x, y, and z
plt.figure(figsize=(10,6))
plt.plot(data['time'], data['xArm'], label='XArm')
plt.plot(data['time'], data['yArm'], label='YArm')
plt.plot(data['time'], data['zArm'], label='ZArm')
plt.plot(data['time'], data['x'], label='X')
plt.plot(data['time'], data['y'], label='Y')
plt.plot(data['time'], data['z'], label='Z')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Trunk Position vs. Time')
plt.legend()
plt.grid(True)
plt.show()
