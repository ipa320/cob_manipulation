#include "cob_kinematics/ik_wrapper.h"

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>

#include <cob_kinematics/GetPositionIKExtended.h>

#include <tf_conversions/tf_kdl.h>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>

class IKWrapperNode{
    boost::scoped_ptr<IKWrapper> wrapper;
    ros::NodeHandle nh;
    
    ros::ServiceClient get_ik_client;
    ros::ServiceClient get_fk_client;
    ros::ServiceClient get_constraint_aware_ik_client;    
    ros::ServiceServer get_ik_server;
    ros::ServiceServer get_fk_server;
    ros::ServiceServer get_constraint_aware_ik_server;    
    ros::ServiceServer get_ik_extended_server;

public:    
    bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
			 kinematics_msgs::GetPositionIK::Response &response){
	response.error_code.val  = wrapper->transformPositionIKRequest(request.ik_request);
	if(response.error_code.val == 0) return get_ik_client.call(request, response);
	return true;
    }
    
    bool getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request, 
			 kinematics_msgs::GetConstraintAwarePositionIK::Response &response){
	response.error_code.val  = wrapper->transformPositionIKRequest(request.ik_request);
	if(response.error_code.val == 0) return get_constraint_aware_ik_client.call(request, response);
	return true;
    }

    bool getPositionIKExtended(cob_kinematics::GetPositionIKExtended::Request &request, 
			 cob_kinematics::GetPositionIKExtended::Response &response){

	bool ret= true;
	const geometry_msgs::Pose *pose=0;
	if(request.ik_pose.orientation.x != 0 || request.ik_pose.orientation.y != 0 || request.ik_pose.orientation.z != 0 || request.ik_pose.orientation.w != 0)
	    pose = &request.ik_pose;
	response.error_code.val  = wrapper->transformPositionIKRequest(request.ik_request, pose);
	
	if(response.error_code.val == 0) {
	    if(request.use_constraints){
		kinematics_msgs::GetConstraintAwarePositionIK::Request request_intern;
		kinematics_msgs::GetConstraintAwarePositionIK::Response response_intern;
		request_intern.ik_request = request.ik_request;
		request_intern.timeout = request.timeout;
		request_intern.constraints = request.constraints;
		ret = get_constraint_aware_ik_client.call(request_intern, response_intern);
		response.solution = response_intern.solution;
		response.error_code = response_intern.error_code;
	    }else{
		kinematics_msgs::GetPositionIK::Request request_intern;
		kinematics_msgs::GetPositionIK::Response response_intern;
		request_intern.ik_request = request.ik_request;
		request_intern.timeout = request.timeout;
		ret = get_ik_client.call(request_intern, response_intern);
		response.solution = response_intern.solution;
		response.error_code = response_intern.error_code;
	    }
	}
	return ret;
    }


    bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                     kinematics_msgs::GetPositionFK::Response &response){
	return wrapper->getPositionFK(request, response, get_fk_client);
    }

    IKWrapperNode():nh("~"){
	std::string prefix, ik_name, fk_name, contraint_ik_name, info_name,link_names, prefetch_tips;
	
	nh.param<std::string>("srvs_prefix", prefix, "/cob_arm_kinematics/");    
	nh.param<std::string>("ik_srv", ik_name, prefix+"get_ik");    
	nh.param<std::string>("fk_srv", fk_name, prefix+"get_fk");    
	nh.param<std::string>("contraint_aware_srv", contraint_ik_name, prefix +"get_constraint_aware_ik");    
	nh.param<std::string>("info_srv", info_name, prefix +"get_ik_solver_info");

	nh.param<std::string>("link_names", link_names, "");
	nh.param<std::string>("prefetch_tips", prefetch_tips, "");
	
	std::vector<std::string> root_names;
	if(!root_names.empty())
	    boost::split(root_names,link_names, boost::is_any_of(" ,"), boost::token_compress_on);

	std::vector<std::string> tip_names;
	if(!prefetch_tips.empty())
	    boost::split(tip_names,prefetch_tips, boost::is_any_of(" ,"), boost::token_compress_on);

	std::cout << ik_name << std::endl;
	get_ik_client = ros::service::createClient<kinematics_msgs::GetPositionIK>(ik_name);    
	get_constraint_aware_ik_client = ros::service::createClient<kinematics_msgs::GetConstraintAwarePositionIK>(contraint_ik_name);    
	get_fk_client = ros::service::createClient<kinematics_msgs::GetPositionFK>(fk_name);

	bool okay;
	for(int i=0; i<20; ++i){
	    okay = true;
	    if (get_ik_client.exists()) break;
	    if (get_constraint_aware_ik_client.exists()) break;
	    if (get_fk_client.exists()) break;
	    ros::Duration(1.0).sleep();
	    okay = false;
	}

	kinematics_msgs::GetKinematicSolverInfo::Request info_request;
	kinematics_msgs::GetKinematicSolverInfo::Response info_response;
	if(!ros::service::call(info_name, info_request, info_response))
	    ROS_WARN("Info service not available!");
	
	root_names.insert(root_names.end(), info_response.kinematic_solver_info.link_names.begin(), info_response.kinematic_solver_info.link_names.end());

	if(!root_names.empty() && okay){
	    urdf::Model model;
	    model.initParam("robot_description");
	    wrapper.reset(new IKWrapper(model, root_names, tip_names));
	    if(get_ik_client.exists()){
		 get_ik_server = nh.advertiseService("get_ik", &IKWrapperNode::getPositionIK,this);
	    }
	    if(get_constraint_aware_ik_client.exists()){
		 get_constraint_aware_ik_server = nh.advertiseService("get_constraint_aware_ik", &IKWrapperNode::getConstraintAwarePositionIK,this);
	    }
	    if(get_fk_client.exists()){
		get_fk_server = nh.advertiseService("get_fk", &IKWrapperNode::getPositionFK,this);
	    }
	    if(get_ik_client.exists() || get_constraint_aware_ik_client.exists()){
		 get_ik_extended_server = nh.advertiseService("get_ik_extended", &IKWrapperNode::getPositionIKExtended,this);
	    }
	}else{
	     nh.shutdown();
	 }
    }
};	
	
int main(int argc, char * argv[]){
    ros::init(argc,argv,"ik_wrapper_node");
    IKWrapperNode node;
    ros::spin();
    return 0;
}
