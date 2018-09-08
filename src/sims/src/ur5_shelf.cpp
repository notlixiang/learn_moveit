/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_interface_ur5");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them
  // in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and
  // "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control
  // and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for
  // improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing
  // objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step
  // introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a
  // high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text,
  // cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE,
                           rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to
  // RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s",
                 move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 move_group.getEndEffectorLink().c_str());

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;

  auto current_pose = move_group.getCurrentPose();

  ROS_INFO("current_pose positioin x %f , y %f , z %f ",
           current_pose.pose.position.x, current_pose.pose.position.y,
           current_pose.pose.position.z);
  ROS_INFO("current_pose orientation x %f , y %f , z %f , w %f ",
           current_pose.pose.orientation.x, current_pose.pose.orientation.y,
           current_pose.pose.orientation.z, current_pose.pose.orientation.w);

  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);

  ROS_INFO_NAMED("tutorial", "now add the cabin");
  visual_tools.prompt(
      "Press 'next' to start");
  // define a collision object
  moveit_msgs::CollisionObject collision_object_cabin;
  collision_object_cabin.header.frame_id = move_group.getPlanningFrame();
  collision_object_cabin.id = "cabin";

  // define a cabin shape
  std::string cabin_dir =
      "file:://home/null/ros_ws/learn_moveit/src/moveit_tutorials/doc/"
      "move_group_interface/collision/huojia.stl";
  shapes::Mesh* m = shapes::createMeshFromResource(cabin_dir);
  shape_msgs::Mesh cabin_mesh;
  shapes::ShapeMsg cabin_mesh_msg;
  shapes::constructMsgFromShape(m, cabin_mesh_msg);
  cabin_mesh = boost::get<shape_msgs::Mesh>(cabin_mesh_msg);

  // cabin position
  float cabin_x = 0.4;
  float cabin_y = 0;
  float cabin_z = -0.6;

  geometry_msgs::Pose cabin_pose;
  cabin_pose.orientation.w =  0.707;
  cabin_pose.orientation.z =  0.707;
  cabin_pose.position.x = cabin_x;
  cabin_pose.position.y = cabin_y;
  cabin_pose.position.z = cabin_z;

  collision_object_cabin.meshes.push_back(cabin_mesh);
  collision_object_cabin.mesh_poses.push_back(cabin_pose);
  collision_object_cabin.operation = collision_object_cabin.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object_cabin);
  ROS_INFO("add cabin into the world.");

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object_floor;
  collision_object_floor.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_floor.id = "floor";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 0.01;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.6;

  collision_object_floor.primitives.push_back(primitive);
  collision_object_floor.primitive_poses.push_back(box_pose);
  collision_object_floor.operation = collision_object_floor.ADD;

//  std::vector<moveit_msgs::CollisionObject> collision_objects2;
//  collision_objects2.push_back(collision_object_floor);
  collision_objects.push_back(collision_object_floor);
  ROS_INFO("add floor into the world.");

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object_wall;
  collision_object_wall.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_wall.id = "wall";

  // Define a box to add to the world.
//  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.01;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 1.5;

  // Define a pose for the box (specified relative to frame_id)
//  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.4;
  box_pose.position.y = 0;
  box_pose.position.z = 0.2;

  collision_object_wall.primitives.push_back(primitive);
  collision_object_wall.primitive_poses.push_back(box_pose);
  collision_object_wall.operation = collision_object_wall.ADD;

//  std::vector<moveit_msgs::CollisionObject> collision_objects2;
  collision_objects.push_back(collision_object_wall);
  ROS_INFO("add wall into the world.");

  planning_scene_interface.addCollisionObjects(collision_objects);
//  planning_scene_interface.addCollisionObjects(collision_objects2);
  ros::Duration(1.0).sleep();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//  visual_tools.prompt(
//      "Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = 0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.1;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "SUCCESSED" : "FAILED");
  visual_tools.prompt("Press 'next' to execute plan result");

  if (success) {
    move_group.execute(my_plan);
  }

  // move test
  move_group.setStartState(*move_group.getCurrentState());
  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = 0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = cabin_x + 0.1;
  target_pose1.position.y = cabin_y - 0;
  target_pose1.position.z = cabin_z + 0.4;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "SUCCESSED" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to execute plan result");

  if (success) {
    move_group.execute(my_plan);
  }

  move_group.setStartState(*move_group.getCurrentState());
  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = 0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = cabin_x   + 0.1;
  target_pose1.position.y = cabin_y - 0;
  target_pose1.position.z = cabin_z + 0.45;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "SUCCESSED" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to execute plan result");

  if (success) {
    move_group.execute(my_plan);
  }

  move_group.setStartState(*move_group.getCurrentState());
  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = 0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = cabin_x   + 0.1;
  target_pose1.position.y = cabin_y - 0;
  target_pose1.position.z = cabin_z + 0.5;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "SUCCESSED" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to execute plan result");

  if (success) {
    move_group.execute(my_plan);
  }

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
