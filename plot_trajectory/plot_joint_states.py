import rosbag
import matplotlib.pyplot as plt
import numpy as np


bag = rosbag.Bag('trajectory_positions.bag')


times = []
positions = []
velocities = []

for topic, msg, t in bag.read_messages(topics='/joint_states'):
    times.append(t.to_sec())
    positions.append(msg.position)  
    if len(msg.velocity) == len(msg.position) and len(msg.velocity) > 0:
        velocities.append(msg.velocity)
    else:
        # If no velocity data, append NaNs to keep array shapes consistent
        velocities.append([float('nan')] * len(msg.position))

bag.close()

positions = np.array(positions).T  # shape: [num_joints, num_samples]
velocities = np.array(velocities).T  

# Plotting
plt.figure()
for i in range(positions.shape[0]):
    plt.plot(times, positions[i], label=f'Joint {i+1}')
plt.xlabel('Time [s]')
plt.ylabel('Joint Position [rad]')
plt.title('Joint Positions over Time')
plt.legend()


# Plot joint velocities
plt.figure()
for i in range(velocities.shape[0]):
    plt.plot(times, velocities[i], label=f'Joint {i+1}')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.title('Joint Velocities')
plt.legend()

plt.tight_layout()
plt.show()
