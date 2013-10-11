#include  <ros/ros.h>
#include  <actionlib/server/simple_action_server.h>
#include  <cob_lookat_action/LookAtAction.h>
#include  <actionlib/client/simple_action_client.h>
#include  <actionlib/client/terminal_state.h>
#include  <control_msgs/FollowJointTrajectoryAction.h>
#include  <trajectory_msgs/JointTrajectory.h>
#include  <trajectory_msgs/JointTrajectoryPoint.h>
#include  <geometry_msgs/PoseStamped.h>
#include  <moveit_msgs/GetPositionIK.h>

#include  <tf/transform_datatypes.h>
#include  <math.h>


class  CobLookAtAction
{
protected:

    ros::NodeHandle  nh;
    
    ros::ServiceClient  ik_client;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  torso_ac;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  head_ac;
    
    //  NodeHandle  instance  must  be  created  before  this  line.  Otherwise  strange  error  may  occur.
    actionlib::SimpleActionServer<cob_lookat_action::LookAtAction>  lookat_as;  
    std::string  lookat_action_name;
    //  create  messages  that  are  used  to  published  feedback/result
    cob_lookat_action::LookAtFeedback  lookat_fb;
    cob_lookat_action::LookAtResult  lookat_res;
    
public:

    CobLookAtAction(std::string  name)  :
        torso_ac(nh,"/torso_controller/follow_joint_trajectory",  true),
        head_ac(nh,"/head_controller/follow_joint_trajectory",  true),
        lookat_as(nh,  name,  boost::bind(&CobLookAtAction::goalCB,  this,  _1),  false),
        lookat_action_name(name)
    {
        ik_client  =  nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
        
	ROS_WARN("Waiting for Torso-AS");
        torso_ac.waitForServer();
	ROS_WARN("Waiting for Head-AS");
        head_ac.waitForServer();

        lookat_as.start();
    }

    ~CobLookAtAction(void)
    {
    }

    void  goalCB(const  cob_lookat_action::LookAtGoalConstPtr  &goal)
    {
        //  helper  variables
        bool  success  =  false;
        
        //  publish  info  to  the  console  for  the  user
        ROS_INFO("%s:  Executing!",  lookat_action_name.c_str());
        
        moveit_msgs::GetPositionIK  srv;
        srv.request.ik_request.group_name  =  "lookat";
        srv.request.ik_request.ik_link_name  =  "lookat_focus_frame";
        srv.request.ik_request.timeout  =  ros::Duration(1.0);
        srv.request.ik_request.attempts  =  1;
        srv.request.ik_request.pose_stamped  =  goal->target;
        
        std::vector<std::string>  joint_names;
        joint_names.push_back("torso_lower_neck_tilt_joint");
        joint_names.push_back("torso_pan_joint");
        joint_names.push_back("torso_upper_neck_tilt_joint");
        joint_names.push_back("lookat_lin_joint");        
        joint_names.push_back("lookat_x_joint");
        joint_names.push_back("lookat_y_joint");
        joint_names.push_back("lookat_z_joint");
        std::vector<double>  joint_seed  (7,  0.0);
        
        moveit_msgs::RobotState  seed;
        seed.joint_state.header  =  goal->target.header;
        seed.joint_state.name  =  joint_names;
        seed.joint_state.position  =  joint_seed;
        srv.request.ik_request.robot_state  =  seed;

    if  (ik_client.call(srv))
    {
        ROS_DEBUG("IK  Result:  %d",  srv.response.error_code.val);
        if(srv.response.error_code.val  ==  1)
        {
            ROS_INFO("IK  Solution  found");
            success  =  true;
        }
        else
        {
            ROS_ERROR("No  IK  Solution  found");
            success  =  false;
            lookat_res.success  =  success;
            //  set  the  action  state  to  aborted
            lookat_as.setAborted(lookat_res);
            return;
        }
    }
    else
    {
        ROS_ERROR("IK-Call  failed");
        success  =  false;
        lookat_res.success  =  success;
        //  set  the  action  state  to  aborted
        lookat_as.setAborted(lookat_res);
        return;
    }
    
    ROS_INFO("Config  found:  ");
    std::vector<double>  torso_config;
    torso_config.resize(3);
    std::vector<double>  head_config;
    head_config.resize(1);
    for(unsigned  int  i=0;  i<srv.response.solution.joint_state.name.size();  i++)
    {
        ROS_DEBUG("%s:  %f",  srv.response.solution.joint_state.name[i].c_str(),  srv.response.solution.joint_state.position[i]);
        if(srv.response.solution.joint_state.name[i]  ==  "torso_lower_neck_tilt_joint")
            torso_config[0]  =  srv.response.solution.joint_state.position[i];
        if(srv.response.solution.joint_state.name[i]  ==  "torso_pan_joint")
            torso_config[1]  =  srv.response.solution.joint_state.position[i];
        if(srv.response.solution.joint_state.name[i]  ==  "torso_upper_neck_tilt_joint")
            torso_config[2]  =  srv.response.solution.joint_state.position[i];
        if(srv.response.solution.joint_state.name[i]  ==  "lookat_lin_joint")
            if(srv.response.solution.joint_state.position[i] >= 0)
                head_config[0] = 0.0;   //look backwards
            else
                head_config[0] = -3.1415926;    //lock forwards
    }
    
    //  send  a  goal  to  the  action
    control_msgs::FollowJointTrajectoryGoal  torso_goal;
    torso_goal.trajectory.header.stamp  =  ros::Time::now();
    torso_goal.trajectory.header.frame_id  =  "base_link";
    torso_goal.trajectory.joint_names.push_back("torso_lower_neck_tilt_joint");
    torso_goal.trajectory.joint_names.push_back("torso_pan_joint");
    torso_goal.trajectory.joint_names.push_back("torso_upper_neck_tilt_joint");
    trajectory_msgs::JointTrajectoryPoint  torso_point;
    torso_point.positions  = torso_config;
    torso_point.time_from_start  = ros::Duration(3.0);
    torso_goal.trajectory.points.push_back(torso_point);
    
    control_msgs::FollowJointTrajectoryGoal  head_goal;
    head_goal.trajectory.header.stamp  =  ros::Time::now();
    head_goal.trajectory.header.frame_id  =  "base_link";
    head_goal.trajectory.joint_names.push_back("head_axis_joint");
    trajectory_msgs::JointTrajectoryPoint  head_point;
    head_point.positions = head_config;
    head_point.time_from_start = ros::Duration(3.0);
    head_goal.trajectory.points.push_back(head_point);
    
    torso_ac.sendGoal(torso_goal);
    head_ac.sendGoal(head_goal);

    //wait  for  the  action  to  return
    bool  finished_before_timeout  =  torso_ac.waitForResult(ros::Duration(30.0)) && head_ac.waitForResult(ros::Duration(30.0));

    if  (finished_before_timeout)
    {
        ROS_INFO("Both Actions  finished");
        success  =  true;
    }
    else
    {
        ROS_INFO("At leas one Action  did  not  finish  before  timeout.");
        success  =  false;
    }
    
    
    if(success)
        {
        ROS_INFO("%s:  Succeeded",  lookat_action_name.c_str());
        lookat_res.success  =  success;
        
        //  set  the  action  state  to  succeeded
        lookat_as.setSucceeded(lookat_res);
    }
    else
    {
        ROS_INFO("%s:  Failed  to  execute",  lookat_action_name.c_str());
        lookat_res.success  =  success;
        
        //  set  the  action  state  to  aborted
        lookat_as.setAborted(lookat_res);
    }
    }
  
};


int  main(int  argc,  char**  argv)
{
    ros::init(argc,  argv,  "cob_lookat_action");

    CobLookAtAction  lookat("lookat_action");
    ros::spin();

    return  0;
}
