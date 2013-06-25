#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_pick_place_action/COBPickUpAction.h>
#include <cob_pick_place_action/cob_pick_place_action.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "cob_pick_action_client");

  // create the action client
  // true causes the client to spin its own thread
  static const std::string COB_PICKUP_ACTION = "COBPickUp"; 
  actionlib::SimpleActionClient<cob_pick_place_action::COBPickUpAction> ac(COB_PICKUP_ACTION, true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  cob_pick_place_action::COBPickUpGoal goal;
  
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(1000.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  

  //exit
  return 0;
}
