#include <ros/ros.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>

#include <pluginlib/class_loader.h>
#include <kinematics_base/kinematics_base.h>

#include <tf/transform_listener.h>

bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                            geometry_msgs::PoseStamped &pose_msg_out, 
                            const std::string &root_frame, 
                            const tf::TransformListener &tf)
{
    if( root_frame == pose_msg.header.frame_id){
        pose_msg_out = pose_msg;
        return true;
    }
    geometry_msgs::PoseStamped pose_msg_in = pose_msg;
    ROS_DEBUG("Request:\nframe_id: %s\nPosition: %f %f %f\n:Orientation: %f %f %f %f\n",
                pose_msg_in.header.frame_id.c_str(),
                pose_msg_in.pose.position.x,
                pose_msg_in.pose.position.y,
                pose_msg_in.pose.position.z,
                pose_msg_in.pose.orientation.x,
                pose_msg_in.pose.orientation.y,
                pose_msg_in.pose.orientation.z,
                pose_msg_in.pose.orientation.w);
    tf::Stamped<tf::Pose> pose_stamped;
    poseStampedMsgToTF(pose_msg_in, pose_stamped);

    if (!tf.canTransform(root_frame, pose_stamped.frame_id_, pose_stamped.stamp_))
    {
        std::string err;    
        if (tf.getLatestCommonTime(pose_stamped.frame_id_, root_frame, pose_stamped.stamp_, &err) != tf::NO_ERROR)
        {
        ROS_ERROR("Cannot transform from '%s' to '%s'. TF said: %s",pose_stamped.frame_id_.c_str(),root_frame.c_str(), err.c_str());
        return false;
        }
    }    
    try
    {
        tf.transformPose(root_frame, pose_stamped, pose_stamped);
    }
    catch(...)
    {
        ROS_ERROR("Cannot transform from '%s' to '%s'",pose_stamped.frame_id_.c_str(),root_frame.c_str());
        return false;
    } 
    tf::poseStampedTFToMsg(pose_stamped,pose_msg_out);   
    return true;
}
  
arm_navigation_msgs::ArmNavigationErrorCodes kinematicsErrorCodeToMotionPlanningErrorCode(const int &kinematics_error_code)
{
arm_navigation_msgs::ArmNavigationErrorCodes error_code;

if(kinematics_error_code == kinematics::SUCCESS)
    error_code.val = error_code.SUCCESS;
else if(kinematics_error_code == kinematics::TIMED_OUT)
    error_code.val = error_code.TIMED_OUT;
else if(kinematics_error_code == kinematics::NO_IK_SOLUTION)
    error_code.val = error_code.NO_IK_SOLUTION;
else if(kinematics_error_code == kinematics::FRAME_TRANSFORM_FAILURE)
    error_code.val = error_code.FRAME_TRANSFORM_FAILURE;
else if(kinematics_error_code == kinematics::IK_LINK_INVALID)
    error_code.val = error_code.INVALID_LINK_NAME;
else if(kinematics_error_code == kinematics::IK_LINK_IN_COLLISION)
    error_code.val = error_code.IK_LINK_IN_COLLISION;
else if(kinematics_error_code == kinematics::STATE_IN_COLLISION)
    error_code.val = error_code.COLLISION_CONSTRAINTS_VIOLATED;
else if(kinematics_error_code == kinematics::INVALID_LINK_NAME)
    error_code.val = error_code.INVALID_LINK_NAME;
else if(kinematics_error_code == kinematics::GOAL_CONSTRAINTS_VIOLATED)
    error_code.val = error_code.GOAL_CONSTRAINTS_VIOLATED;
else if(kinematics_error_code == kinematics::INACTIVE)
    error_code.val = 0;
return error_code;
}

class KinematicsNode
{
  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
  kinematics::KinematicsBase* kinematics_solver_;

  ros::NodeHandle node_handle_;
  ros::ServiceServer ik_service_, fk_service_;
  
  std::string root_name_;
  bool active_;

  tf::TransformListener tf_listener_;
  
public:

  KinematicsNode():kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),node_handle_("~"){
    std::string group_name, kinematics_solver_name, joint_states;
    node_handle_.param<std::string>("group", group_name, std::string());
    node_handle_.param<std::string>("kinematics_solver",kinematics_solver_name," ");
    node_handle_.param<std::string>("joint_states",joint_states,"joints_states");
    ROS_INFO("Using kinematics solver name: %s",kinematics_solver_name.c_str());
    if (group_name.empty())
    {
        ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to monitor, node cannot compute collision aware inverse kinematics");
        active_ = false;
        return;
    }

    kinematics_solver_ = NULL;
    try
    {
        kinematics_solver_ = kinematics_loader_.createClassInstance(kinematics_solver_name);
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
        active_ = false;
        return;
    }
    if(kinematics_solver_->initialize(group_name))
        active_ = true;
    else
    {
        active_ = false;
        return;
    }
    root_name_ = kinematics_solver_->getBaseFrame();
    ik_service_ = node_handle_.advertiseService("get_ik",&KinematicsNode::getPositionIK,this);
    fk_service_ = node_handle_.advertiseService("get_fk",&KinematicsNode::getPositionFK,this);

 }
  bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                     kinematics_msgs::GetPositionIK::Response &response){
    if(!active_)
    {
        ROS_ERROR("IK service not active");
        return false;
    }
    geometry_msgs::PoseStamped pose_msg_out;
    
    convertPoseToRootFrame(request.ik_request.pose_stamped, pose_msg_out, root_name_, tf_listener_);

    request.ik_request.pose_stamped = pose_msg_out;
    ROS_DEBUG_STREAM("Pose is " << pose_msg_out.pose.position.x << " " << pose_msg_out.pose.position.y << " " << pose_msg_out.pose.position.z);
    
    if(request.ik_request.ik_seed_state.joint_state.position.size() == 0){
        ROS_ERROR("Invalid robot state in IK request");
        response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
        return true;
    }
    
    int kinematics_error_code;
    bool ik_valid = kinematics_solver_->searchPositionIK(pose_msg_out.pose,
                                                        request.ik_request.ik_seed_state.joint_state.position,
                                                        request.timeout.toSec(),
                                                        response.solution.joint_state.position,
                                                        kinematics_error_code);

    response.error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);

    if(ik_valid)
    {
        response.solution.joint_state.name = request.ik_request.ik_seed_state.joint_state.name;
        response.error_code.val = response.error_code.SUCCESS;
        return true;
    }
    else
    {
        ROS_DEBUG("An IK solution could not be found");   
        return true;
    }
  }

  bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                     kinematics_msgs::GetPositionFK::Response &response){
    if(!active_)
    {
        ROS_ERROR("FK service not active");
        return false;
    }

    response.pose_stamped.resize(request.fk_link_names.size());
    response.fk_link_names.resize(request.fk_link_names.size());

    std::vector<geometry_msgs::Pose> solutions;
    solutions.resize(request.fk_link_names.size());
    if(kinematics_solver_->getPositionFK(request.fk_link_names,request.robot_state.joint_state.position,solutions) >=0)
    {    
        for(unsigned int i=0; i < solutions.size(); i++)
        {      
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = solutions[i];
            pose_stamped.header.frame_id = root_name_;
            pose_stamped.header.stamp = ros::Time();
            
            if(!convertPoseToRootFrame(pose_stamped, response.pose_stamped[i], request.header.frame_id, tf_listener_)) {
                response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
                break;
            }
            response.fk_link_names[i] = request.fk_link_names[i];
            response.error_code.val = response.error_code.SUCCESS;
        }
    }
    else
    {
        ROS_ERROR("Could not compute FK");
        response.error_code.val = response.error_code.NO_FK_SOLUTION;
    }
    return true;
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_node");

  KinematicsNode node;

  ros::spin();

  return(0);
}
