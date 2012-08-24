/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author Sachin Chitta, Ioan Sucan
 *********************************************************************/

#include "planning_environment/monitors/planning_monitor.h"

#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <arm_navigation_msgs/SyncPlanningSceneAction.h>
#include <actionlib/client/simple_action_client.h>
#include <planning_environment/models/model_utils.h>

static const std::string SYNC_PLANNING_SCENE_NAME ="sync_planning_scene";
static const unsigned int UNSUCCESSFUL_REPLY_LIMIT = 5;
static const ros::Duration PLANNING_SCENE_CLIENT_TIMEOUT(5.0);

namespace planning_environment
{
class EnvironmentServer
{
public:	
  EnvironmentServer() :
  private_handle_("~")
  {
    std::string robot_description_name = root_handle_.resolveName("robot_description", true);

    collision_models_ = new planning_environment::CollisionModels(robot_description_name);
    planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
    planning_monitor_->setUseCollisionMap(true);

    planning_monitor_->waitForState();
    planning_monitor_->startEnvironmentMonitor();
    
    register_planning_scene_service_ = root_handle_.advertiseService("register_planning_scene", &EnvironmentServer::registerPlanningScene, this);
    get_robot_state_service_ = private_handle_.advertiseService("get_robot_state", &EnvironmentServer::getRobotState, this);
    get_planning_scene_service_ = private_handle_.advertiseService("get_planning_scene", &EnvironmentServer::getPlanningScene, this);
    set_planning_scene_diff_service_ = private_handle_.advertiseService("set_planning_scene_diff", &EnvironmentServer::setPlanningSceneDiff, this);

    ROS_INFO("Environment server started");
  }
	
  virtual ~EnvironmentServer()
  {
    for(std::map<std::string, actionlib::SimpleActionClient<arm_navigation_msgs::SyncPlanningSceneAction>* >::iterator it = sync_planning_scene_clients_.begin();
        it != sync_planning_scene_clients_.end();
        it++) {
      delete it->second;
    }
    delete collision_models_;
    if(planning_monitor_) {
      delete planning_monitor_;
    }
  }
	
private:
  
  bool registerPlanningScene(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res)
  {
    register_lock_.lock();
    if(req.__connection_header->find("callerid") == req.__connection_header->end()) {
      ROS_ERROR_STREAM("Request has no callerid");
      return false;
    }
    std::string callerid = req.__connection_header->find("callerid")->second;
    if(sync_planning_scene_clients_.find(callerid) != sync_planning_scene_clients_.end()) {
      delete sync_planning_scene_clients_[callerid];
    }
    sync_planning_scene_clients_[callerid] = new actionlib::SimpleActionClient<arm_navigation_msgs::SyncPlanningSceneAction>(callerid+"/"+SYNC_PLANNING_SCENE_NAME, true);
    if(!sync_planning_scene_clients_[callerid]->waitForServer(ros::Duration(10.0))) {
      ROS_INFO_STREAM("Couldn't connect back to action server for " << callerid << ". Removing from list");
      delete sync_planning_scene_clients_[callerid];
      sync_planning_scene_clients_.erase(callerid);
      register_lock_.unlock();
      return false;
    } 
    ROS_INFO_STREAM("Successfully connected to planning scene action server for " << callerid);
    register_lock_.unlock();
    return true;
  }
		
  bool getRobotState(arm_navigation_msgs::GetRobotState::Request &req, 
                     arm_navigation_msgs::GetRobotState::Response &res)
  {
    planning_monitor_->getCurrentRobotState(res.robot_state);
    return true;    
  }

  bool getPlanningScene(arm_navigation_msgs::GetPlanningScene::Request &req, 
                        arm_navigation_msgs::GetPlanningScene::Response &res) 
  {
    planning_monitor_->getCompletePlanningScene(req.planning_scene_diff,
                                              req.operations,
                                              res.planning_scene);
    return true;
  }

  bool setPlanningSceneDiff(arm_navigation_msgs::SetPlanningSceneDiff::Request &req, 
                            arm_navigation_msgs::SetPlanningSceneDiff::Response &res) 
  {
    ros::WallTime s1 = ros::WallTime::now();
    planning_monitor_->getCompletePlanningScene(req.planning_scene_diff,
                                              req.operations,
                                              res.planning_scene);

    arm_navigation_msgs::SyncPlanningSceneGoal planning_scene_goal;
    planning_scene_goal.planning_scene = res.planning_scene;
    for(std::map<std::string, actionlib::SimpleActionClient<arm_navigation_msgs::SyncPlanningSceneAction>* >::iterator it = sync_planning_scene_clients_.begin();
        it != sync_planning_scene_clients_.end();
        it++) {
      it->second->sendGoal(planning_scene_goal);
    }
    std::vector<std::string> bad_list;
    for(std::map<std::string, actionlib::SimpleActionClient<arm_navigation_msgs::SyncPlanningSceneAction>* >::iterator it = sync_planning_scene_clients_.begin();
        it != sync_planning_scene_clients_.end();
        it++) {
      if(!it->second->waitForResult(PLANNING_SCENE_CLIENT_TIMEOUT)) {
        unsuccessful_planning_scene_client_replies_[it->first]++;
        ROS_INFO_STREAM("Did not get reply from planning scene client " << it->first 
                        << ".  Incrementing counter to " << unsuccessful_planning_scene_client_replies_[it->first]);
        if(unsuccessful_planning_scene_client_replies_[it->first] > UNSUCCESSFUL_REPLY_LIMIT) {
          ROS_WARN_STREAM("Failed to get reply from planning scene client " << it->first << " for " 
                          << UNSUCCESSFUL_REPLY_LIMIT << " consecutive tries.  Removing");
          bad_list.push_back(it->first);
        } 
        continue;
      } 
      unsuccessful_planning_scene_client_replies_[it->first] = 0;
    }

    for(unsigned int i = 0; i < bad_list.size(); i++) {
      delete sync_planning_scene_clients_[bad_list[i]];
      sync_planning_scene_clients_.erase(bad_list[i]);
    }
    ROS_DEBUG_STREAM("Setting planning scene diff took " << (ros::WallTime::now()-s1).toSec());
    return true;
  }

private:

  boost::mutex register_lock_;
  ros::NodeHandle root_handle_, private_handle_;
  planning_environment::CollisionModels *collision_models_;
  planning_environment::PlanningMonitor *planning_monitor_;
  tf::TransformListener tf_;	

  ros::ServiceServer get_robot_state_service_;
  ros::ServiceServer get_planning_scene_service_;
  ros::ServiceServer set_planning_scene_diff_service_;

  ros::ServiceServer register_planning_scene_service_;
  std::map<std::string, unsigned int> unsuccessful_planning_scene_client_replies_;
  std::map<std::string, actionlib::SimpleActionClient<arm_navigation_msgs::SyncPlanningSceneAction>* > sync_planning_scene_clients_;
};    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "environment_server");
  //figuring out whether robot_description has been remapped

  ros::AsyncSpinner spinner(1); 
  spinner.start();
  planning_environment::EnvironmentServer environment_monitor;
  ros::waitForShutdown();
  return 0;
}
