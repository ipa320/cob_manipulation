#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_pick_place_action/CobPlaceAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "cob_place_action_client_node");

  // create the action client
  // true causes the client to spin its own thread
  static const std::string COB_PLACE_ACTION_NAME = "cob_place_action"; 
  actionlib::SimpleActionClient<cob_pick_place_action::CobPlaceAction> ac(COB_PLACE_ACTION_NAME, true);

  ROS_INFO("Waiting for action server to start...");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  cob_pick_place_action::CobPlaceGoal goal;
  goal.object_id = 11;
  goal.object_name = "sauerkraut";
  
  /// Fill in pose from object_detection
  //goal.destination.header.stamp = ros::Time::now();
  //goal.destination.header.frame_id = "odom_combined";
  //goal.destination.pose.position.x -0.5;
  //goal.destination.pose.position.y = -0.5;  
  //goal.destination.pose.position.z = 0.6;
  
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
