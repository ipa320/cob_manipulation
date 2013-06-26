/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_manipulation
 * \note
 *   ROS package name: cob_pick_place_action
 *
 * \author
 *   Author: Rohit Chandra, email:rohit.chandra@ipa.fhg.de
 *
 * \date Date of creation: March, 2013
 *
 * \brief
 *	 This package provides pick place action
 *   It takes object id and choosed grasp from the graspList. 
 *	 It does pick and place depending on the request
 *
 ****************************************************************/
#ifndef COB_PICK_ACTION_H
#define COB_PICK_ACTION_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <actionlib/server/simple_action_server.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <cob_pick_place_action/CobPickAction.h>
#include <cob_pick_place_action/CobPlaceAction.h>
#include <GraspTable.h>



class CobPickPlaceActionServer
{
private:
	ros::NodeHandle nh_;
	
	ros::Publisher pub_co; //publisher for collision_objects
	ros::Publisher pub_ao; //publisher for attached_collision_objects
	
	boost::scoped_ptr<actionlib::SimpleActionServer<cob_pick_place_action::CobPickAction> > as_pick;
	boost::scoped_ptr<actionlib::SimpleActionServer<cob_pick_place_action::CobPlaceAction> > as_place;
	
	moveit::planning_interface::MoveGroup group;
	
	char* GraspTableIniFile;
	GraspTable* m_GraspTable;
	
public:
	CobPickPlaceActionServer() : group("arm") {}
	~CobPickPlaceActionServer();
	
	void initialize();
	void run();

	void pick_goal_cb(const cob_pick_place_action::CobPickGoalConstPtr &goal);
	void place_goal_cb(const cob_pick_place_action::CobPlaceGoalConstPtr &goal);
	
	void setUpEnvironment(std::string object_name, geometry_msgs::PoseStamped object_pose);
	void detachObject(std::string object_name);
	
	void fillAllGrasps(unsigned int objectClassId, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps);
	void fillSingleGrasp(unsigned int objectClassId, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps);
	
	sensor_msgs::JointState MapHandConfiguration(sensor_msgs::JointState table_config);
	geometry_msgs::Pose transformPose(geometry_msgs::Pose grasp_pose_wrt_object, geometry_msgs::Pose object_pose);
	manipulation_msgs::GripperTranslation calculateApproachDirection(std::vector<double> current_hand_pose, std::vector<double> current_hand_pre_pose);

};
#endif

