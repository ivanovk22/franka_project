import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

############ Plots the desired and the execited trajectories ############
########### Needs robot_traj.npz, which is created from get_robot_traj.py  ###########

# Configure the center according to the robot's start position for better visualization
radius = 0.1  # 10 cm
# the real start pose is x = 0.306891, y = -0.0, z = 0.486882 (taken from the execution of cicrle_trajectory.cpp)
# x is shifter with 0.1(the radius) to match the robot's circle center. The robot draws the circle
# from where it is initially positioned, so actually the x coorddinate of the
# center of the circle is x - radius
center = [0.206891, 0.0, 0.486882]
theta = np.linspace(0, 2*np.pi, 300)

x1 = center[0] + radius * np.cos(theta)
y1 = center[1] + radius * np.sin(theta)
z1 = np.ones_like(theta) * center[2]  # constant height


# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x1, y1, z1, label='Desirec Trajectory', color='blue')
ax.set_zlim(0.48, 0.65)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Circumference in XY-plane')
ax.legend()


# Load data
data = np.load('robot_traj.npz')
x2 = data['x']
y2 = data['y']
z2 = data['z']
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x2, y2, z2, label='Executed trajectory')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Circumference in XY-plane')
ax.legend()
plt.show()