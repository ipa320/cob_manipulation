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
 *   Author: Rohit Chandra, email:Rohit.Chandra@ipa.fraunhofer.de
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

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
//#include <geometry_msgs/Quaternion.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <cob_pick_place_action/CobPickAction.h>
#include <cob_pick_place_action/CobPlaceAction.h>
#include <cob_grasp_generation/QueryGraspsAction.h>
#include <GraspTable.h>



class CobPickPlaceActionServer
{
private:
	ros::NodeHandle nh_;
	
	ros::Publisher pub_co; //publisher for collision_objects
	ros::Publisher pub_ao; //publisher for attached_collision_objects

	boost::scoped_ptr<actionlib::SimpleActionServer<cob_pick_place_action::CobPickAction> > as_pick;
	boost::scoped_ptr<actionlib::SimpleActionServer<cob_pick_place_action::CobPlaceAction> > as_place;
	
	boost::scoped_ptr<actionlib::SimpleActionClient<cob_grasp_generation::QueryGraspsAction> > ac_grasps_or;
	
	moveit::planning_interface::MoveGroup group;
	
	char* GraspTableIniFile;
	GraspTable* m_GraspTable;
	
	bool last_grasp_valid;
	std::string last_object_name;
	manipulation_msgs::Grasp last_grasp;
	tf::TransformListener tf_listener_;
	
	std::map<unsigned int,std::string> map_classid_to_classname;
	
public:
	CobPickPlaceActionServer() : group("arm") {}
	~CobPickPlaceActionServer();
	
	void initialize();
	void run();

	void pick_goal_cb(const cob_pick_place_action::CobPickGoalConstPtr &goal);
	void place_goal_cb(const cob_pick_place_action::CobPlaceGoalConstPtr &goal);

	void insertObject(std::string object_name, geometry_msgs::PoseStamped object_pose);
	
	void fillAllGraspsKIT(unsigned int objectClassId, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps);
	void fillSingleGraspKIT(unsigned int objectClassId, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps);
	void convertGraspKIT(Grasp* current_grasp, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps);
	
	void fillGraspsOR(unsigned int objectClassId, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps);
	
	sensor_msgs::JointState MapHandConfiguration(sensor_msgs::JointState table_config);
	tf::Transform transformPose(tf::Transform transform_O_from_SDH, tf::Transform transform_HEADER_from_OBJECT, std::string object_frame_id);
	manipulation_msgs::GripperTranslation calculateApproachDirection(geometry_msgs::Pose msg_pose_grasp_FOOTPRINT_from_ARM7, geometry_msgs::Pose msg_pose_pre_FOOTPRINT_from_ARM7);

};
#endif

