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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <cob_pick_place_action/COBPickUpAction.h>
#include "geometry_msgs/Quaternion.h"
#include <tf/tf.h>
#include <GraspTable.h>
#include <tf/transform_datatypes.h>


class COBPickAction
{
private:
	ros::NodeHandle nh_;
    boost::scoped_ptr<actionlib::SimpleActionServer<cob_pick_place_action::COBPickUpAction> > cob_pick_action_server;
	ros::Publisher pub_co ;
	ros::Publisher attached_object_publisher ;
	moveit::planning_interface::MoveGroup group;
	char* GraspTableIniFile;
	GraspTable* m_GraspTable;
	
public:
	COBPickAction() : group("arm") {}
	void initialize();
	//~ COBPickAction();
	~COBPickAction();
	void goalCB(const cob_pick_place_action::COBPickUpGoalConstPtr &goal);
	void setUpEnvironment();
	//~ void fillGrasps( std::vector<manipulation_msgs::Grasp> &grasps);
	void fillGrasps(unsigned int objectClassId, std::vector<manipulation_msgs::Grasp> &grasps);
	void setCOBPickupResponse(cob_pick_place_action::COBPickUpResult &action_res, bool success);
	void Run();
	geometry_msgs::Pose GraspPoseWRTBaseFootprint(geometry_msgs::Pose grasp_pose, geometry_msgs::Pose pose_of_object_recognition);
	sensor_msgs::JointState MapHandConfiguration(sensor_msgs::JointState table_config);
	manipulation_msgs::GripperTranslation getGraspApproachData(std::vector<double> current_hand_pose, std::vector<double> current_hand_pre_pose);

};
#endif

