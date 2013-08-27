#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cob_lookat_action/LookAtAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionIK.h>

#include <tf/transform_datatypes.h>
#include <math.h>


class CobLookAtAction
{
protected:

  ros::NodeHandle nh_;
  
  ros::ServiceClient ik_client;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_client;

  
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<cob_lookat_action::LookAtAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  cob_lookat_action::LookAtFeedback feedback_;
  cob_lookat_action::LookAtResult result_;
  

public:

  CobLookAtAction(std::string name) :
    trajectory_client(nh_,"/torso_controller/follow_joint_trajectory", true),
    as_(nh_, name, boost::bind(&CobLookAtAction::goalCB, this, _1), false),
    action_name_(name)
  {
    ik_client = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    
    trajectory_client.waitForServer();

    as_.start();
  }

  ~CobLookAtAction(void)
  {
  }

  void goalCB(const cob_lookat_action::LookAtGoalConstPtr &goal)
  {
    // helper variables
    bool success = false;
    
    // publish info to the console for the user
    ROS_INFO("%s: Executing!", action_name_.c_str());
    
    moveit_msgs::GetPositionIK srv;
    
    srv.request.ik_request.group_name = "lookat";
    srv.request.ik_request.ik_link_name = "lookat_focus_frame";
    srv.request.ik_request.timeout = ros::Duration(1.0);
    srv.request.ik_request.attempts = 1;
    
    srv.request.ik_request.pose_stamped = goal->target;
    
    std::vector<std::string> joint_names;
    joint_names.push_back("torso_lower_neck_tilt_joint");
    joint_names.push_back("torso_pan_joint");
    joint_names.push_back("torso_upper_neck_tilt_joint");
    joint_names.push_back("lookat_back_joint");    
    joint_names.push_back("lookat_x_joint");
    joint_names.push_back("lookat_y_joint");
    joint_names.push_back("lookat_z_joint");
    std::vector<double> joint_seed (7, 0.0);
    
    moveit_msgs::RobotState seed;
    seed.joint_state.header = goal->target.header;
    seed.joint_state.name = joint_names;
    seed.joint_state.position = joint_seed;
    
    srv.request.ik_request.robot_state = seed;

	if (ik_client.call(srv))
	{
		ROS_INFO("IK Result: %d", srv.response.error_code.val);
		if(srv.response.error_code.val == 1)
		{
			ROS_INFO("IK Solution found");
			success = true;
		}
		else
		{
			ROS_ERROR("No IK Solution found");
			success = false;
			result_.success = success;
			// set the action state to aborted
			as_.setAborted(result_);
			return;
		}
	}
	else
	{
		ROS_ERROR("IK-Call failed");
		success = false;
		result_.success = success;
		// set the action state to aborted
		as_.setAborted(result_);
		return;
	}
	
	ROS_INFO("Config found: ");
	std::vector<double> torso_config;
	torso_config.resize(3);
	for(unsigned int i=0; i<srv.response.solution.joint_state.name.size(); i++)
	{
		ROS_INFO("%s: %f", srv.response.solution.joint_state.name[i].c_str(), srv.response.solution.joint_state.position[i]);
		if(srv.response.solution.joint_state.name[i] == "torso_lower_neck_tilt_joint")
			torso_config[0] = srv.response.solution.joint_state.position[i];
		if(srv.response.solution.joint_state.name[i] == "torso_pan_joint")
			torso_config[1] = srv.response.solution.joint_state.position[i];
		if(srv.response.solution.joint_state.name[i] == "torso_upper_neck_tilt_joint")
			torso_config[2] = srv.response.solution.joint_state.position[i];
	}
	
	
	
	// send a goal to the action
	control_msgs::FollowJointTrajectoryGoal torso_goal;
	torso_goal.trajectory.header.stamp = ros::Time::now();
	torso_goal.trajectory.header.frame_id = "base_link";
	torso_goal.trajectory.joint_names.push_back("torso_lower_neck_tilt_joint");
	torso_goal.trajectory.joint_names.push_back("torso_pan_joint");
	torso_goal.trajectory.joint_names.push_back("torso_upper_neck_tilt_joint");
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions = torso_config;
	point.time_from_start = ros::Duration(3.0);
	torso_goal.trajectory.points.push_back(point);
	
	
	
	trajectory_client.sendGoal(torso_goal);

	//wait for the action to return
	bool finished_before_timeout = trajectory_client.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = trajectory_client.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		success = true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		success = false;
	}
	
	
	if(success)
    {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		result_.success = success;
		
		// set the action state to succeeded
		as_.setSucceeded(result_);
	}
	else
	{
		ROS_INFO("%s: Failed to execute", action_name_.c_str());
		result_.success = success;
		
		// set the action state to aborted
		as_.setAborted(result_);
	}
  }
 
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_lookat");

  CobLookAtAction lookat(ros::this_node::getName());
  ros::spin();

  return 0;
}
