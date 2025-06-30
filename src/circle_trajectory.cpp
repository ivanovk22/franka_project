#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "circle_trajectory_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Move group for panda arm
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");


  move_group.setEndEffectorLink("panda_hand_tcp");  // This is the TCP for Franka

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.loadMarkerPub("/rviz_visual_tools");
  visual_tools.setBaseFrame("panda_link0");
  visual_tools.enableBatchPublishing();
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();

  ROS_INFO("Waiting for RViz to subscribe to /rviz_visual_tools...");
  ros::Duration(5).sleep();
  
  move_group.setPlanningTime(10.0);

  // Example: Create a circular trajectory in XY-plane
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
  ROS_INFO("Start pose position: [x: %f, y: %f, z: %f]",
          start_pose.position.x,
          start_pose.position.y,
          start_pose.position.z);


  double radius = 0.1; // 10cm
  int num_points = 300;
  // In order to start the trajectory from the start_pose we need to calculate where the center should be
  // which is done by subtracting the radius from the x coordinate
  double center_x = start_pose.position.x - radius;
  double center_y = start_pose.position.y;

  for (int i = 1; i < num_points; ++i)
  {
    double angle = 2 * M_PI * i / num_points;
    geometry_msgs::Pose target = start_pose;
    target.position.x = center_x + radius * cos(angle);
    target.position.y = center_y + radius * sin(angle);
    waypoints.push_back(target);
  }
  
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.trigger();


  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  // Compute Cartesian path
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  ROS_INFO("Path computed with %.2f%% success", fraction * 100.0);


  // Execute the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  move_group.execute(plan);

  ros::shutdown();
  return 0;
}
