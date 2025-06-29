import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting


radius = 0.1  # 10 cm
center = [0.206891, 0.0, 0.486882]  # in robot base frame
theta = np.linspace(0, 2*np.pi, 300)


for t in theta:
    x = center[0] + radius * np.cos(t)
    y = center[1] + radius * np.sin(t)
    z = center[2]
    



x1 = center[0] + radius * np.cos(theta)
y1 = center[1] + radius * np.sin(theta)
z1 = np.ones_like(theta) * center[2]  # constant height
# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x1, y1, z1, label='Desirec Trajectory', color='blue')
# ax.set_xlim(-0.2, 0.2)
# ax.set_ylim(-0.2, 0.2)
ax.set_zlim(0.48, 0.65)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Circumference in XY-plane')
ax.legend()
data = np.load('robot_traj.npz')
x = data['x']
y = data['y']
z = data['z']
# for i in range(10):
#     print(f'x1: {x1[i]}, x2:{x[i]}')


# Load arrays



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='Executed trajectory')
# ax.set_xlim(-0.2, 0.2)
# ax.set_ylim(-0.2, 0.2)
# ax.set_zlim(0, 0.8)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()