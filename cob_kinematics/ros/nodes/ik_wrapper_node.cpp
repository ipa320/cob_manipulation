/*!
 *****************************************************************
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   ROS stack name: cob_manipulation
 * \note
 *   ROS package name: cob_kinematics
 *
 * \author
 *   Author: Mathias Lüdtke
 *
 * \brief
 *   IK/FK service wrapper node with support for multiple IK links at the end-effector
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "cob_kinematics/ik_wrapper.h"

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>

#include <cob_kinematics/GetPositionIKExtended.h>

#include <tf_conversions/tf_kdl.h>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>

#include <planning_environment/models/collision_models_interface.h>

#include <std_srvs/Empty.h>

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

    planning_environment::CollisionModelsInterface cmi;
public:    
    void updateRobotState(const arm_navigation_msgs::PlanningScene &scene){
	if(wrapper) wrapper->updateRobotState(scene.robot_state);	
    }
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
            if(request.constraint_aware){
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

    IKWrapperNode():nh("~"),cmi("/robot_description",false){
	
    cmi.addSetPlanningSceneCallback(boost::bind(&IKWrapperNode::updateRobotState,this, _1));
    std::string prefix, ik_name, fk_name, constraint_ik_name, info_name,link_names, prefetch_tips;

    nh.param<std::string>("srvs_prefix", prefix, "/cob_arm_kinematics/");    
    nh.param<std::string>("ik_srv", ik_name, prefix+"get_ik");    
    nh.param<std::string>("fk_srv", fk_name, prefix+"get_fk");    
    nh.param<std::string>("constraint_aware_srv", constraint_ik_name, prefix +"get_constraint_aware_ik");    
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
    get_constraint_aware_ik_client = ros::service::createClient<kinematics_msgs::GetConstraintAwarePositionIK>(constraint_ik_name);    
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
        ros::NodeHandle rh;
        urdf::Model model;
        model.initParam("robot_description");
        wrapper.reset(new IKWrapper(model, root_names, tip_names));
        if(get_ik_client.exists()){
         get_ik_server = rh.advertiseService("get_ik", &IKWrapperNode::getPositionIK,this);
        }
        if(get_constraint_aware_ik_client.exists()){
         get_constraint_aware_ik_server = rh.advertiseService("get_constraint_aware_ik", &IKWrapperNode::getConstraintAwarePositionIK,this);
        }
        if(get_fk_client.exists()){
        get_fk_server = rh.advertiseService("get_fk", &IKWrapperNode::getPositionFK,this);
        }
        if(get_ik_client.exists() || get_constraint_aware_ik_client.exists()){
         get_ik_extended_server = rh.advertiseService("get_ik_extended", &IKWrapperNode::getPositionIKExtended,this);
        }
	if(ros::service::waitForService("/register_planning_scene", ros::Duration(10.0))){
	    std_srvs::Empty::Request req;
	    std_srvs::Empty::Response res;
	    ros::service::call("/register_planning_scene", req, res);
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
