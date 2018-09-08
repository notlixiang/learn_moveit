/************************
*move group for ur5
*Author: yee
*Date: 2018-03-31
*************************/

//collision object from stl
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc,char **argv){
    ros::init(argc,argv,"planning_in_cabin");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static std::string PLANNING_GROUP="manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool success;
     

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup *joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
 
    namespace rvt=rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose=Eigen::Affine3d::Identity();
    text_pose.translation().z()=0.75;
    visual_tools.publishText(text_pose," motion planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    //define a collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id=move_group.getPlanningFrame();
    collision_object.id="cabin";

    //define a cabin shape
    std::string cabin_dir="file:://home/yee/ur_ws/src/ur5_planning/collision/cabin4.stl";
    shapes::Mesh* m=shapes::createMeshFromResource(cabin_dir);
    shape_msgs::Mesh cabin_mesh;
    shapes::ShapeMsg cabin_mesh_msg;
    shapes::constructMsgFromShape(m,cabin_mesh_msg);
    cabin_mesh=boost::get<shape_msgs::Mesh>(cabin_mesh_msg);

    //cabin position
    float cabin_x=0.5;
    float cabin_y=0;
    float cabin_z=-0.3;

    geometry_msgs::Pose cabin_pose;
    cabin_pose.orientation.w=0;//0.707;
    cabin_pose.orientation.z=1;//0.707;
    cabin_pose.position.x=cabin_x;
    cabin_pose.position.y=cabin_y;
    cabin_pose.position.z=cabin_z;

    collision_object.meshes.push_back(cabin_mesh);
    collision_object.mesh_poses.push_back(cabin_pose);
    collision_object.operation=collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("add an object into the world.");
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep();


    //define pose goal
    geometry_msgs::Pose start,pose1,pose2;
    start.orientation.w=1;
    start.position.x=cabin_x-0.1;
    start.position.y=cabin_y=0;
    start.position.z=cabin_z+0.5;

    pose1.orientation.w= 1.0;
    pose1.position.x = cabin_x+0.1;
    pose1.position.y = cabin_y-0.1;
    pose1.position.z = cabin_z+0.44;

    pose2.orientation.w= 1.0;
    pose2.position.x = cabin_x+0.3;
    pose2.position.y = cabin_y-0.1;
    pose2.position.z = cabin_z+0.44;

    move_group.setPoseTarget(start);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED"); 
    if(success){
        visual_tools.publishAxisLabeled(start, "start");//coordinates with 3 axis
        visual_tools.publishText(text_pose,"move to start ", rvt::WHITE,rvt::XLARGE);
        move_group.execute(my_plan);
    }
    visual_tools.trigger();
    visual_tools.prompt("next");//its feasible

    //pose1
    move_group.setPoseTarget(pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED"); 
    if(success){
        visual_tools.publishText(text_pose,"move to pose1 ", rvt::WHITE,rvt::XLARGE);
        move_group.execute(my_plan);
    }
    //move back
    move_group.setPoseTarget(start);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED"); 
    if(success){
        move_group.execute(my_plan);
    }
    visual_tools.trigger();
    visual_tools.prompt("next");//its feasible

    //pose2
    move_group.setPoseTarget(pose2);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED");
    if(success){
        visual_tools.publishText(text_pose,"move to pose2 ", rvt::WHITE,rvt::XLARGE);
        move_group.execute(my_plan);
    }
    //move back
    move_group.setPoseTarget(start);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED"); 
    if(success){
        move_group.execute(my_plan);
    }
    visual_tools.trigger();
    visual_tools.prompt("next");//its feasible


    //release memory
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    //planning_scene_interface.removeCollisionObjects(object_ids);
    visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}