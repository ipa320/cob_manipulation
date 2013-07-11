#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_pick_place_action/CobPickAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "cob_pick_action_client_node");

  // create the action client
  // true causes the client to spin its own thread
  static const std::string COB_PICKUP_ACTION_NAME = "cob_pick_action"; 
  actionlib::SimpleActionClient<cob_pick_place_action::CobPickAction> ac(COB_PICKUP_ACTION_NAME, true);

  ROS_INFO("Waiting for action server to start...");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  
  ///send a goal to the action
  cob_pick_place_action::CobPickGoal goal;
  //goal.object_id = 11;
  //goal.object_name = "sauerkraut";
  goal.object_id = 18;
  goal.object_name = "yellowsaltcube";
  //goal.object_id = 65;
  //goal.object_name = "fruitdrink";

  goal.object_pose.header.stamp = ros::Time::now();
  goal.object_pose.header.frame_id = "/base_footprint";
  goal.object_pose.pose.position.x = -0.5;
  goal.object_pose.pose.position.y = -0.5;  
  goal.object_pose.pose.position.z =  0.6;
  goal.object_pose.pose.orientation.w = 1.0;
  goal.object_pose.pose.orientation.x = 0.0;
  goal.object_pose.pose.orientation.y = 0.0;
  goal.object_pose.pose.orientation.z = 0.0;
  
  ac.sendGoal(goal);

  //wait for the action to return (5 min)
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  
  //exit
  return 0;
}
