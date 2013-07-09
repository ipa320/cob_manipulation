#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_pick_place_action/CobPickAction.h>
#include <cob_pick_place_action/CobCollisionObjectAction.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "cob_pick_action_client_node");
  bool finished_before_timeout = false;
  
  // create the action client
  // true causes the client to spin its own thread
  static const std::string COB_PICKUP_ACTION_NAME = "cob_pick_action"; 
  actionlib::SimpleActionClient<cob_pick_place_action::CobPickAction> ac_pick(COB_PICKUP_ACTION_NAME, true);
  
  static const std::string COB_COLLISION_OBJECT_NAME = "cob_collision_object_action";
  actionlib::SimpleActionClient<cob_pick_place_action::CobCollisionObjectAction> ac_collision(COB_COLLISION_OBJECT_NAME, true);

  ROS_INFO("Waiting for action server to start...");
  // wait for the action server to start
  ac_pick.waitForServer(); //will wait for infinite time
  ac_collision.waitForServer(); //will wait for infinite time

  //*******************************************************************
  // Add support surface
  //*******************************************************************
  ROS_INFO("Action server started, sending support surface goal.");
  ///send a goal to the action
  cob_pick_place_action::CobCollisionObjectGoal goal_collision;
  
  goal_collision.object.header.stamp = ros::Time::now();
  goal_collision.object.header.frame_id = "base_footprint";
  goal_collision.object.id = "support_surface";
  goal_collision.object.operation = goal_collision.object.REMOVE;

  goal_collision.object.primitives.resize(1);
  goal_collision.object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  
  //goal_collision.object.primitives[0].dimensions.resize(
  //  shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  goal_collision.object.primitives[0].dimensions.resize(3);
  goal_collision.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  goal_collision.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  goal_collision.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.8;

  goal_collision.object.primitive_poses.resize(1);
  goal_collision.object.primitive_poses[0].position.x = -0.9;
  goal_collision.object.primitive_poses[0].position.y = 0.0;
  goal_collision.object.primitive_poses[0].position.z = 0.4;
  goal_collision.object.primitive_poses[0].orientation.x = 0.0;
  goal_collision.object.primitive_poses[0].orientation.y = 0.0;
  goal_collision.object.primitive_poses[0].orientation.z = 0.0;
  goal_collision.object.primitive_poses[0].orientation.w = 1.0;

  ac_collision.sendGoal(goal_collision); // Remove
  finished_before_timeout = ac_collision.waitForResult(ros::Duration(300.0));
  actionlib::SimpleClientGoalState state_coll_remove = ac_collision.getState();
  if (finished_before_timeout)
    ROS_INFO("Action finished: %s",state_coll_remove.toString().c_str());
  else
    ROS_INFO("Action timeout occured: %s",state_coll_remove.toString().c_str());	  
  
  goal_collision.object.operation = goal_collision.object.ADD;
  ac_collision.sendGoal(goal_collision); // Add
  finished_before_timeout = ac_collision.waitForResult(ros::Duration(300.0));
  actionlib::SimpleClientGoalState state_coll_add = ac_collision.getState();
  if (finished_before_timeout)
    ROS_INFO("Action finished: %s",state_coll_add.toString().c_str());
  else
    ROS_INFO("Action timeout occured: %s",state_coll_add.toString().c_str());
    
  //*******************************************************************
  // Send Pick goal
  //*******************************************************************
  ROS_INFO("Action server started, sending pick goal.");
  ///send a goal to the action
  cob_pick_place_action::CobPickGoal goal_pick;
  //goal_pick.object_id = 11;
  //goal_pick.object_name = "sauerkraut";
  goal_pick.object_id = 13;
  goal_pick.object_name = "fruittea";
  //goal_pick.object_id = 18;
  //goal_pick.object_name = "yellowsaltcube";
  //goal_pick.object_id = 65;
  //goal_pick.object_name = "fruitdrink";

  goal_pick.object_pose.header.stamp = ros::Time::now();
  goal_pick.object_pose.header.frame_id = "/base_footprint";
  goal_pick.object_pose.pose.position.x = -0.7;
  goal_pick.object_pose.pose.position.y = -0;  
  goal_pick.object_pose.pose.position.z =  0.85;
  goal_pick.object_pose.pose.orientation.w = 1.0;
  goal_pick.object_pose.pose.orientation.x = 0.0;
  goal_pick.object_pose.pose.orientation.y = 0.0;
  goal_pick.object_pose.pose.orientation.z = 0.0;
  
  ac_pick.sendGoal(goal_pick);

  //wait for the action to return (5 min)
  finished_before_timeout = ac_pick.waitForResult(ros::Duration(300.0));
  actionlib::SimpleClientGoalState state_pick = ac_pick.getState();
  if (finished_before_timeout)
  {
    ROS_INFO("Action finished: %s",state_pick.toString().c_str());
  }
  
  //exit
  return 0;
}
