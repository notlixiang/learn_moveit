

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <time.h>
#define random(x) (rand() % x)
int main(int argc, char** argv) {
  ros::init(argc, argv, "right_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr kinematic_state(
      new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  const robot_state::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const std::vector<std::string>& joint_names =
      joint_model_group->getJointModelNames();
  robot_state::RobotState& current_state =
      planning_scene.getCurrentStateNonConst();
  // collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the
  // right arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  visualization_msgs::Marker points, line_strip, line_list, pointsb, pointsc;
  points.header.frame_id = line_strip.header.frame_id =
      line_list.header.frame_id = "/base_link";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp =
      ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action =
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w =
      line_list.pose.orientation.w = 1.0;

  pointsb.header.frame_id = "/base_link";
  pointsb.header.stamp = ros::Time::now();
  pointsb.ns = "pointsb";
  pointsb.action = visualization_msgs::Marker::ADD;
  pointsb.pose.orientation.w = 1.0;

  pointsc.header.frame_id = "/base_link";
  pointsc.header.stamp = ros::Time::now();
  pointsc.ns = "pointsc";
  pointsc.action = visualization_msgs::Marker::ADD;
  pointsc.pose.orientation.w = 1.0;
  points.id = 0;
  pointsb.id = 1;
  pointsc.id = 2;

  // ros::Publisher pub =
  // n.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);
  //  ros::Duration(1.0).sleep();

  //  // primitive sphere
  //  moveit_msgs::CollisionObject col_sphere_msg;
  //  col_sphere_msg.id = "sphere_primitive";
  //  col_sphere_msg.header.frame_id = "/base_link";
  //  col_sphere_msg.operation = moveit_msgs::CollisionObject::ADD;

  //  shapes::Mesh* m =
  //  shapes::createMeshFromResource("package://motion_plan/new2.stl");
  //  shape_msgs::Mesh co_mesh;
  //  shapes::ShapeMsg co_mesh_msg;
  //  shapes::constructMsgFromShape(m,co_mesh_msg);
  //  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  //  col_sphere_msg.meshes.resize(1);
  //  col_sphere_msg.meshes[0] = co_mesh;
  //    col_sphere_msg.mesh_poses.resize(1);
  //    col_sphere_msg.mesh_poses[0].position.x = 0.0;
  //     col_sphere_msg.mesh_poses[0].position.y = 0.0;
  //     col_sphere_msg.mesh_poses[0].position.z = -0.5;
  //     col_sphere_msg.mesh_poses[0].orientation.w= 1.0;
  //     col_sphere_msg.mesh_poses[0].orientation.x= 0.0;
  //     col_sphere_msg.mesh_poses[0].orientation.y= 0.0;
  //    col_sphere_msg.mesh_poses[0].orientation.z= 0.0;
  // pub.publish(col_sphere_msg);

  //   col_sphere_msg.meshes.push_back(co_mesh);
  // col_sphere_msg.mesh_poses.push_back(col_sphere_msg.mesh_poses[0]);

  // pub.publish(col_sphere_msg);
  // ros::Duration(0.1).sleep();

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(col_sphere_msg);

  // planning_scene_interface.addCollisionObjects(collision_objects);

  points.type = visualization_msgs::Marker::POINTS;
  pointsb.type = visualization_msgs::Marker::POINTS;
  pointsc.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  pointsb.scale.x = 0.01;
  pointsb.scale.y = 0.01;
  pointsc.scale.x = 0.01;
  pointsc.scale.y = 0.01;

  // Points are green
  points.color.b = 1.0f;
  points.color.a = 1.0;
  pointsb.color.b = 0.5f;
  pointsb.color.g = 1.0f;
  pointsb.color.a = 1.0;
  pointsc.color.r = 1.0f;
  //    pointsc.color.g = 1.0f;
  //      pointsc.color.b = 1.0f;
  pointsc.color.a = 1.0;
  int sample = 150;
  int samplerotate = 4;
  int i;
  ///////////////////////////////////////////////////////////////////////////mazhuoming
  // double uplimit=0.589+ col_sphere_msg.mesh_poses[0].position.z;
  // double lowlimit=0.115+ col_sphere_msg.mesh_poses[0].position.z;
  // for(i=0;i<sample;i++)
  //{double theta=((double)random(2000)/(double)1000-1)*(3.14/2);
  // double r=((double)random(1000)/(double)1000)*(0.64-0.3)+0.3;
  // double zz=((double)random(1000)/(double)1000)*(uplimit-lowlimit)+lowlimit;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////v
  // double uplimit=0.35+ col_sphere_msg.mesh_poses[0].position.z;//hemingda
  // double lowlimit=0.166+ col_sphere_msg.mesh_poses[0].position.z;
  // for(i=0;i<sample;i++)
  //{double theta=((double)random(2000)/(double)1000-1)*(4.71/2)-1.57;
  // double r=((double)random(1000)/(double)1000)*(0.85-0.149)+0.149;
  // double zz=((double)random(1000)/(double)1000)*(uplimit-lowlimit)+lowlimit;
  // double xx=r*cos(theta);
  // double yy=r*sin(theta);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////v
  double uplimit = 1.1;  // hemingda
  double lowlimit = -1.1;
  for (i = 0; i < sample; i++) {
    double theta = 0;
    double r = ((double)random(1000) / (double)1000) * (1.3 - 0.1) + 0.1;
    double zz =
        ((double)random(1000) / (double)1000) * (uplimit - lowlimit) + lowlimit;
    double xx = r * cos(theta);
    double yy = r * sin(theta);

    bool found_ik = false;
    ///////////////////////////////////////////////////////////////////////////
    int k = 0;
    int count = 0;
    double result = 0.0;
    double lenth = 0.4;
    double a[4][3] = {{1.555, 0.012, -0.000},
                      {-2.118, 1.552, 2.594},
                      {-0.016, -0.001, -1.571},
                      {-3.126, 0.018, 1.571}};
    double b[4][3] = {{xx - lenth, yy, zz},
                      {xx, yy, zz + lenth},
                      {xx, yy + lenth, zz},
                      {xx, yy - lenth, zz}};
    for (; k < samplerotate; k++) {
      //    double rr=((double)random(1000)/(double)1000)*3.14;//
      //    double gg=((double)random(1000)/(double)1000)*(1.57);//0~0.5pi

      //    double bb=((double)random(1000)/(double)1000)*(3.14-(-3.14))-3.14;
      geometry_msgs::Pose target_pose1;
      double rr = a[k][0];  // r
      double gg = a[k][1];  // p
      double bb = a[k][2];  // y

      target_pose1.position.x = b[k][0];
      target_pose1.position.y = b[k][1];
      target_pose1.position.z = b[k][2];

      //    ROS_INFO("   %lf::::%lf   ",xx ,b[k][0]);
      //    ROS_INFO("   %lf::::%lf   ",yy,b[k][1]);
      //    ROS_INFO("   %lf::::%lf   ",zz,b[k][2]);
      target_pose1.orientation =
          tf::createQuaternionMsgFromRollPitchYaw(rr, gg, bb);

      found_ik = kinematic_state->setFromIK(
          joint_model_group, target_pose1, 10,
          0.05);  // 10 means The number of times IK is attempted
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

      //  current_state.setJointGroupPositions(joint_model_group, joint_values);
      //   planning_scene.checkSelfCollision(collision_request,
      //   collision_result);
      //   if(collision_result.collision==true )
      //    {    collision_result.clear();//!!!!!!!!!!!!!!!!!!!!!!!!very very
      //    very important!!!!
      //   continue;}

      if (found_ik) {
        count = count + 1;
      }

      //   std::string movegroup;
      //      movegroup = "manipulator";
      //   std::string refFrame2;
      //   refFrame2="base_link";
      //              do{
      //              result = mv.mPlanning(movegroup,target_pose1);
      //              ROS_INFO("The result of motion planning is %ld", (long
      //              int)result);
      //              } while(!result);
      //      ros::WallDuration(2.0).sleep();
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //  moveit::planning_interface::MoveGroup group("manipulator");

      //  ros::Publisher display_publisher
      //  =n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",
      //  1, true);
      //  moveit_msgs::DisplayTrajectory display_trajectory;
      //  moveit::planning_interface::MoveGroup::Plan my_plan;
      //  group.setPoseTarget(target_pose1);
      //  bool success = group.plan(my_plan);//.asyncMove();//here we do not use
      //  group.plan(my_plan)
      //  //because this is not a block order

      //  /* Sleep to give Rviz time to visualize the plan. */
      // if(success)
      //{
      //    count=count+1;
      //}
    }
    result = count / (double)samplerotate;
    ROS_INFO("Visualizing plan   %d:%f", i, result);
    if (result == 1) {
      geometry_msgs::Point p;
      p.x = xx;
      p.y = yy;
      p.z = zz;

      pointsb.points.push_back(p);

      for (double i = 0; i < 1.57; i += 0.2) {
        p.x = xx * cos(i);
        p.y = xx * sin(i) * (-1);
        p.z = zz;
        pointsb.points.push_back(p);
      }
      for (double i = 0; i < 1.57; i += 0.2) {
        p.x = xx * cos(i);
        p.y = xx * sin(i);
        p.z = zz;
        pointsb.points.push_back(p);
      }
    }

    else {
      geometry_msgs::Point p;
      p.x = xx;
      p.y = yy;
      p.z = zz;

      pointsc.points.push_back(p);
      for (double i = 0; i < 1.57; i += 0.2) {
        p.x = xx * cos(i);
        p.y = xx * sin(i) * (-1);
        p.z = zz;
        pointsc.points.push_back(p);
      }
      for (double i = 0; i < 1.57; i += 0.2) {
        p.x = xx * cos(i);
        p.y = xx * sin(i);
        p.z = zz;
        pointsc.points.push_back(p);
      }
    }
  }

  ROS_INFO_STREAM("over");
  while (ros::ok()) {
    marker_pub.publish(points);
    marker_pub.publish(pointsb);
    marker_pub.publish(pointsc);
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
