# Executing a Trajectory with the Franka Robot in Gazebo

This repository provides a complete ROS-Gazebo simulation for the **Franka Panda robot**,
demonstrating how to plan and execute a **circular end-effector trajectory** using MoveIt!

### Structure

- **launch/**: Starts the robot, simulation, and visualization tools.
- **src/**: The circle_trajectory.cpp which executes the trajectory.
- **plot_trajectory/**: Python scripts for saving and plotting the trajectories + already created .bat file, .npz and plots
- **CMake.txt**
- **package.xml**

### How to run
```bash
- cd ~/ws_moveit/src
'Download the code and paste it in /ws_moveit/src or clone it with:'
- git clone https://github.com/ivanovk22/franka_project.git
- cd ~/ws_moveit
- catkin build
- source ~/ws_moveit/devel/setup.bash
- roslaunch franka_project gazebo.launch

Open new terminal and run:
- rosrun franka_project circle_trajectory_node
```

### How data collection and plotting can be done
After having the package already installed:
```bash
'Open terminal 1:'
- cd ~/ws_moveit/src/franka_project/plot_trajectory
- rosbag record -O filename.bat \tf \tf_static

'Open terminal 2:'
- roslaunch franka_project gazebo.launch

'Open terminal 3:'
- rosrun franka_project circle_trajectory_node

'After compilation of terminal 3, stop the rosbag in terminal 1 with Ctrl + C.
Then stop the roslaunch process in terminal 2.'

'In terminal 2:'
- roscore
'In terminal 3:'
- python3 ~/ws_moveit/src/franka_project/plot_trajectory/get_robot_traj.py
'In terminal 1:'
- rosbag play filename.bat

'After compilation of rosbag play, do Ctrl+C in terminal 3(stop the python script)
The python script creates a .npz file(the name can be changed from the code, default is robot_traj.npz)'

'In order to plot the trajectories:'
'Remark: If you create a new .npz file, change the name in the plot_trajectory.py to the new .npz file name(line 35)'
- cd ~/ws_moveit/src/franka_project/plot_trajectory
- python3 plot_trajectories.py
```

