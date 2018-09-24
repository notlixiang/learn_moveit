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
#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>
#include <sensor_msgs/JointState.h>
//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/collision_detection/Collision.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>

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

std::vector<std::string> joint_names;
std::vector<double> joint_positions;

void statesMessageReceived(const sensor_msgs::JointState current_state) {
  joint_names.clear();
  joint_positions.clear();
  for (int i = 0; i < 19; i++) {
    joint_names.push_back(current_state.name[i]);
    joint_positions.push_back(current_state.position[i]);
  }
}

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

  move_group.setPlannerId("RRTstarkConfigDefault");
  move_group.setPlanningTime(1);
  // move_group.setPlannerId("InformedRRTstarConfigDefault");
  // move_group.setPlannerId("RRTConnectkConfigDefault");

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
  visual_tools.prompt("Press 'next' to start");
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
  cabin_pose.orientation.w = 0.707;
  cabin_pose.orientation.z = 0.707;
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

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object_base;
  collision_object_base.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_base.id = "base";

  // Define a box to add to the world.
  //  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.6;

  // Define a pose for the box (specified relative to frame_id)
  //  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.3;

  collision_object_base.primitives.push_back(primitive);
  collision_object_base.primitive_poses.push_back(box_pose);
  collision_object_base.operation = collision_object_base.ADD;

  //  std::vector<moveit_msgs::CollisionObject> collision_objects2;
  collision_objects.push_back(collision_object_base);
  ROS_INFO("add base into the world.");

  planning_scene_interface.addCollisionObjects(collision_objects);
  planning_scene_interface.applyCollisionObjects(collision_objects);
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

  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = 0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = cabin_x + 0.0;
  target_pose1.position.y = cabin_y - 0;
  target_pose1.position.z = cabin_z + 0.4;
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

  planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");
  //  planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr =
  //  planning_scene_monitor::PlanningSceneMonitorPtr(
  //      new planning_scene_monitor::PlanningSceneMonitor("robot_description",
  //      tf, "name"));

  monitor_ptr->requestPlanningSceneState("get_planning_scene");
  planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
  ps->getCurrentStateNonConst().update();
  planning_scene::PlanningScenePtr scene = ps->diff();
  scene->decoupleParent();

  //  robot_model_loader::RobotModelLoader
  //  robot_model_loader("robot_description");
  //  robot_model::RobotModelPtr kinematic_model =
  //  robot_model_loader.getModel();
  //  planning_scene::PlanningScene planning_scene(kinematic_model);

  //  collision_detection::CollisionRequest collision_request;
  //  collision_detection::CollisionResult collision_result;

  int ran_cnt = 0;
  int gen_cnt = 0;
  for(int iter_main=0;iter_main<1000;iter_main++) {
    ROS_INFO("Randomzing...   %d", ran_cnt++);
    move_group.setStartState(*move_group.getCurrentState());
    move_group.setRandomTarget();
    robot_state::RobotState random_state = move_group.getJointValueTarget();
    scene->setCurrentState(random_state);
    robot_state::RobotState& current_state =
        scene->getCurrentStateNonConst();
    current_state.printStatePositions();
    bool flag = scene->isStateValid(current_state, "manipulator");
    ROS_INFO("isStateValid ? %s", flag ? "yes" : "no");
    if (!flag) {
      continue;
    } else {
      //    move_group.getRandomPose()
      move_group.setPlanningTime(1);
      bool success_find =
          (move_group.plan(my_plan) ==
           moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success_find) {
ran_cnt = 0;
          ROS_INFO("Generating...   %d", gen_cnt++);
        move_group.setPlanningTime(5);
        bool success_optimize =
            (move_group.plan(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_optimize) {
          move_group.execute(my_plan);

          ROS_INFO("trajectory size   %ld",
                   my_plan.trajectory_.joint_trajectory.points.size());

          //        std::vector<std::string> ObjectIds =
          //        scene->getWorld()->getObjectIds();

          //        ROS_INFO("ObjectIds size   %ld", ObjectIds.size());
          //        ROS_INFO("ObjectIds %s %s %s %s", ObjectIds[0].c_str(),
          //                 ObjectIds[1].c_str(), ObjectIds[2].c_str(),
          //                 ObjectIds[3].c_str());

          //        robot_state::RobotState& current_state =
          //            planning_scene.getCurrentStateNonConst();

          std::vector<double> joint_values = {0, 0.0, 0, 0.0, 0.0, 0.0};
          //        const robot_model::JointModelGroup* joint_model_group =
          //            current_state.getJointModelGroup("manipulator");
          //        current_state.setJointGroupPositions(joint_model_group,
          //        joint_values);
          //        current_state.printStatePositions();

          robot_state::RobotState state(scene->getRobotModel());
          state.setJointGroupPositions(joint_model_group, joint_values);
          scene->setCurrentState(state);
          robot_state::RobotState& current_state =
              scene->getCurrentStateNonConst();
          //        current_state.printStatePositions();
          bool flag = scene->isStateValid(current_state, "manipulator");
          ROS_INFO("isStateValid ? %s", flag ? "yes" : "no");

          //        collision_detection::AllowedCollisionMatrix acm =
          //            scene->getAllowedCollisionMatrix();
          //        acm.print(std::cout);
          //        collision_detection::CollisionResult::ContactMap::const_iterator
          //        it2;
          //        for (it2 = collision_result.contacts.begin();
          //             it2 != collision_result.contacts.end(); ++it2) {
          //          acm.setEntry(it2->first.first, it2->first.second, true);
          //        }
          ////        acm.print(std::cout);
          //        collision_result.clear();

          ////        robot_state::RobotState copied_state =
          /// planning_scene.getCurrentState();
          ////        copied_state.printStatePositions();

          //        scene->checkSelfCollision(collision_request,
          //        collision_result,
          //                                          current_state, acm);
          //        ROS_INFO_STREAM("Test 6: Current state is "
          //                        << (collision_result.collision ? "in" : "not
          //                        in")
          //                        << " self collision");

        } else {
          continue;
        }
      } else {
        continue;
      }
    }
  }

  ros::shutdown();
  return 0;
}
