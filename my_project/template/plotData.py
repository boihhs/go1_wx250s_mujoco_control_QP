import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV data
data = pd.read_csv("robot_position.csv")



# Optionally, plot the time histories of x, y, and z
plt.figure(figsize=(10,6))

plt.plot(data['time'], data['xArm'], label='time')






plt.xlabel('Time (s)')
plt.ylabel('time (s)')
plt.title('time vs. Time')
plt.legend()
plt.grid(True)
plt.show()
