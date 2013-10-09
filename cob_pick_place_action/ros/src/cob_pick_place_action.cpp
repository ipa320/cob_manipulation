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
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *   Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
 *
 * \maintainer
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: March, 2013
 *
 * \brief
 *	 This package provides pick place action
 *   It takes object id and choosed grasp from the graspList. 
 *	 It does pick and place depending on the request
 *
 ****************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <cob_pick_place_action/cob_pick_place_action.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <algorithm>

void CobPickPlaceActionServer::initialize()
{
	pub_co = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	pub_ao = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	
	//ToDo: Generate this mapping from GraspTable.txt?
	map_classid_to_classname[11]="sauerkraut";
	map_classid_to_classname[13]="fruittea";
	map_classid_to_classname[16]="orangemarmelade";
	map_classid_to_classname[18]="yellowsaltcube";
	map_classid_to_classname[27]="hotpot";
	map_classid_to_classname[30]="bluesaltcube";
	map_classid_to_classname[31]="yellowsaltcylinder";
	map_classid_to_classname[44]="knaeckebrot";
	map_classid_to_classname[46]="liviosunfloweroil";
	map_classid_to_classname[50]="instantsoup";
	map_classid_to_classname[52]="hotpot2";
	map_classid_to_classname[57]="mueslibars";
	map_classid_to_classname[65]="fruitdrink";
	map_classid_to_classname[99]="ruskwholemeal";
	map_classid_to_classname[101]="koalacandy";
	map_classid_to_classname[103]="instanttomatosoup";
	map_classid_to_classname[106]="bakingsoda";
	map_classid_to_classname[107]="sweetener";
	map_classid_to_classname[109]="chocoicing";
	map_classid_to_classname[119]="tomatoherbsauce";
	map_classid_to_classname[122]="peanuts";
	map_classid_to_classname[126]="herbsalt";
	map_classid_to_classname[128]="bathdetergent";
	
	
	static const std::string COB_PICKUP_ACTION_NAME = "cob_pick_action";
	as_pick.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobPickAction>(nh_, COB_PICKUP_ACTION_NAME, boost::bind(&CobPickPlaceActionServer::pick_goal_cb, this, _1), false));
	as_pick->start();
	
	static const std::string COB_PLACE_ACTION_NAME = "cob_place_action";
	as_place.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobPlaceAction>(nh_, COB_PLACE_ACTION_NAME, boost::bind(&CobPickPlaceActionServer::place_goal_cb, this, _1), false));
	as_place->start();
	
	last_grasp_valid = false;
	last_object_name.clear();
	
	//~ Grasp Table Initializations################################
	std::string path = ros::package::getPath("cob_pick_place_action")+std::string("/files/GraspTable.txt");
	GraspTableIniFile = const_cast<char*>(path.c_str());  
	m_GraspTable = new GraspTable();
	int error = m_GraspTable->Init(GraspTableIniFile);
	
	if(error<0)
		ROS_ERROR("Failed to initialize GraspTables");
	
	
	static const std::string QUERY_GRASPS_OR_ACTION_NAME = "query_grasps";
	ac_grasps_or.reset(new actionlib::SimpleActionClient<cob_grasp_generation::QueryGraspsAction>(nh_, QUERY_GRASPS_OR_ACTION_NAME, true));
	ROS_INFO("Waiting for action server \"%s\" to start...", QUERY_GRASPS_OR_ACTION_NAME.c_str());
	ac_grasps_or->waitForServer(); //will wait for infinite time
	ROS_INFO("Action server \"%s\" started.", QUERY_GRASPS_OR_ACTION_NAME.c_str());
}

void CobPickPlaceActionServer::run()
{
	ROS_INFO("cob_pick_action...spinning");
	ros::spin();
}

void CobPickPlaceActionServer::pick_goal_cb(const cob_pick_place_action::CobPickGoalConstPtr &goal)
{
	ROS_INFO("PickGoalCB: Received new goal: Trying to pick %s", goal->object_name.c_str());
	bool success = false;
	
	ROS_DEBUG_STREAM(*(goal.get()));

	///Updating the object collision_object
	//insertObject(goal->object_name, goal->object_pose);
	
	///Get grasps from corresponding GraspTable
	std::vector<manipulation_msgs::Grasp> grasps;
	if(goal->grasp_database=="KIT")
	{
        ROS_INFO("Using KIT grasp table");
		if(goal->grasp_id!=0)
		{
			ROS_INFO("Using specific grasp_id: %d", goal->grasp_id);
			fillSingleGraspKIT(goal->object_id, goal->grasp_id, goal->object_pose, grasps);
		}
		else
		{
			ROS_INFO("Using all grasps");
			fillAllGraspsKIT(goal->object_id, goal->object_pose, grasps);
		}
	}
	else if(goal->grasp_database=="OpenRAVE")
	{
        ROS_INFO("Using OpenRAVE grasp table");
		fillGraspsOR(goal->object_id, goal->grasp_id, goal->object_pose, grasps);
	}
	else if(goal->grasp_database=="ALL")
	{
        ROS_INFO("Using all available databases");
		std::vector<manipulation_msgs::Grasp> grasps_OR, grasps_KIT;
		fillAllGraspsKIT(goal->object_id, goal->object_pose, grasps_KIT);
		fillGraspsOR(goal->object_id, goal->grasp_id, goal->object_pose, grasps_OR);
		
		grasps = grasps_KIT;
		std::vector<manipulation_msgs::Grasp>::iterator it = grasps.end();
		grasps.insert(it, grasps_OR.begin(), grasps_OR.end());
	}
	else
		ROS_ERROR("Grasp_Database %s not supported! Please use \"KIT\" or \"OpenRAVE\" instead", goal->grasp_database.c_str());

	
	ROS_INFO("PickGoalCB: Found %d grasps for this object", grasps.size());
	for(unsigned int i=0; i<grasps.size(); i++)
	{
		ROS_DEBUG_STREAM("Grasp "<< i << ": " << grasps[i]);
	}
	
	if(!(goal->support_surface.empty()))
	{
		ROS_INFO("Setting SupportSurface to %s", goal->support_surface.c_str());
		group.setSupportSurfaceName(goal->support_surface);
	}
	group.setPlanningTime(300.0);	//default is 5.0 s
	
	
	///Call Pick with result
	manipulation_msgs::Grasp grasp_used;
	success = group.pick(goal->object_name, grasps, grasp_used);
	ROS_DEBUG_STREAM("Executed Grasp: " << grasp_used);
	
	
	///Setting result
	cob_pick_place_action::CobPickResult result;
	std::string response;
	if(success)
	{
		ROS_INFO("Pick was successfull!");
		result.success.data=true;
		response="Hurray!!! Pickup COMPLETE";
		as_pick->setSucceeded(result, response);
		last_grasp_valid = true;
		last_object_name = goal->object_name;
		last_grasp = grasp_used;
	}
	else
	{
		ROS_INFO("Pick failed!");
		result.success.data=false;
		response="Alas!!! Pickup FAILED";
		as_pick->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
	}
}

void CobPickPlaceActionServer::place_goal_cb(const cob_pick_place_action::CobPlaceGoalConstPtr &goal)
{
	ROS_INFO("PlaceCB: Received new goal: Trying to Place %s", goal->object_name.c_str());
	cob_pick_place_action::CobPlaceResult result;
	std::string response;
	bool success = false;
	
	if(!last_grasp_valid || last_object_name.empty() || last_object_name != goal->object_name)
	{
		ROS_ERROR("Object %s cannot be placed", goal->object_name.c_str());
		result.success.data = false;
		response = "Object has not been picked before";
		as_place->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
		return; 
	}
	
	if(goal->destinations.empty())
	{
		ROS_ERROR("Object %s cannot be placed", goal->object_name.c_str());
		result.success.data = false;
		response = "No destinations given";
		as_place->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
		return; 
	}
	
	std::vector<manipulation_msgs::PlaceLocation> locations;
	
	for(unsigned int i=0; i<goal->destinations.size(); i++)
	{
		manipulation_msgs::PlaceLocation place_location;
		
		place_location.id = "Last_"+goal->object_name+"_grasp";
		place_location.post_place_posture = last_grasp.pre_grasp_posture;
		place_location.place_pose = goal->destinations[i];
		place_location.approach.direction.header.frame_id = "/base_footprint";
		place_location.approach.direction.vector.z = -1.0;
		place_location.approach.min_distance = 0.1;
		place_location.approach.desired_distance = 0.15;
		place_location.retreat.direction.header.frame_id = "/base_footprint";
		place_location.retreat.direction.vector.z = 1.0;
		place_location.retreat.min_distance = 0.1;
		place_location.retreat.desired_distance = 0.15;
		
		locations.push_back(place_location);
	}

	group.setPlanningTime(300.0);	//default is 5.0 s
	
	success = group.place(goal->object_name, locations);

	///Setting result
	if(success)
	{
		result.success.data=true;
		response="PLACE SUCCEEDED";
		as_place->setSucceeded(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
	}
	else
	{
		result.success.data=false;
		response="PLACE FAILED";
		as_place->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
	}
}

void CobPickPlaceActionServer::insertObject(std::string object_name, geometry_msgs::PoseStamped object_pose)
{
	ROS_INFO("Adding object to MoveIt! environment..");
	
	moveit_msgs::CollisionObject co;
	co.header.stamp = object_pose.header.stamp;//ros::Time::now();
	co.header.frame_id = object_pose.header.frame_id;
	
	// remove object
	co.id = object_name;
	co.operation = co.REMOVE;
	pub_co.publish(co);
	
	// add object as Mesh
	co.id = object_name;
	co.operation = co.ADD;
	
	std::string mesh_name = object_name;
	std::transform(mesh_name.begin(), mesh_name.end(), mesh_name.begin(), ::tolower);
	
	boost::scoped_ptr<shapes::Mesh> mesh;
	mesh.reset(shapes::createMeshFromResource("package://cob_pick_place_action/files/meshes/"+mesh_name+".stl"));
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(mesh.get(), shape_msg);    
	co.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
	co.mesh_poses.push_back(object_pose.pose);
	pub_co.publish(co);
	
	ros::Duration(1.0).sleep();
}



void CobPickPlaceActionServer::fillAllGraspsKIT(unsigned int objectClassId, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{  
	grasps.clear(); 
	Grasp *current_grasp = NULL;
	
	m_GraspTable->ResetReadPtr(objectClassId);
	unsigned int grasp_index = 0;
	
	current_grasp = m_GraspTable->GetNextGrasp(objectClassId);
	
	while(current_grasp)
	{
		ROS_DEBUG("GraspIndex: %d",grasp_index);
		
		convertGraspKIT(current_grasp, object_pose, grasps);
		
		current_grasp = m_GraspTable->GetNextGrasp(objectClassId);
		grasp_index++;
	}
}

void CobPickPlaceActionServer::fillSingleGraspKIT(unsigned int objectClassId, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{
	grasps.clear(); 
	Grasp *current_grasp = NULL;
	
	current_grasp = m_GraspTable->GetGrasp(objectClassId, grasp_id);
	
	if(current_grasp)
	{
		ROS_INFO("GraspIndex %d found",grasp_id);
		
		convertGraspKIT(current_grasp, object_pose, grasps);
	}
	else
		ROS_ERROR("Grasp %d NOT found", grasp_id);
}

void CobPickPlaceActionServer::convertGraspKIT(Grasp* current_grasp, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{
    bool debug = true;
	manipulation_msgs::Grasp g;
	
	//HandPreGraspConfig
	std::vector<double> current_hand_pre_config = current_grasp->GetHandPreGraspConfig();
	sensor_msgs::JointState pre_grasp_posture;
	pre_grasp_posture.position.clear();
	for (unsigned int i=0; i<current_hand_pre_config.size(); i++)
	{
		pre_grasp_posture.position.push_back(current_hand_pre_config[i]);
	}
	g.pre_grasp_posture = MapHandConfiguration(pre_grasp_posture);
	
	//HandGraspConfig
	std::vector<double> current_hand_config = current_grasp->GetHandOptimalGraspConfig();
	sensor_msgs::JointState grasp_posture;
	grasp_posture.position.clear();
	for (unsigned int i=0; i<current_hand_config.size(); i++)
	{
		grasp_posture.position.push_back(current_hand_config[i]);
	}
	g.grasp_posture= MapHandConfiguration(grasp_posture);
	
	//~~~ TCPGraspPose ~~~
	///GOAL: -> Get TCPGraspPose for arm_7_link wrt base_footprint
	
	// O_from_SDH
	std::vector<double> current_grasp_pose = current_grasp->GetTCPGraspPose();
	tf::Transform transform_grasp_O_from_SDH = tf::Transform(
		tf::createQuaternionFromRPY(current_grasp_pose[3], current_grasp_pose[4], current_grasp_pose[5] ),
		0.001*tf::Vector3(current_grasp_pose[0],current_grasp_pose[1],current_grasp_pose[2]));
	//debug
    if (debug)
    {
        geometry_msgs::Transform msg_grasp_O_from_SDH;
        tf::transformTFToMsg(transform_grasp_O_from_SDH, msg_grasp_O_from_SDH);
        ROS_DEBUG_STREAM("msg_grasp_O_from_SDH:" << msg_grasp_O_from_SDH);
    }
	
	// HEADER_from_O (given)
	tf::Transform transform_HEADER_from_O = tf::Transform(
		tf::Quaternion(object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w),
		tf::Vector3(object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z));
	//debug
    if (debug)
    {
        geometry_msgs::Transform msg_HEADER_from_O;
        tf::transformTFToMsg(transform_HEADER_from_O, msg_HEADER_from_O);
        ROS_DEBUG_STREAM("msg_HEADER_from_O:" << msg_HEADER_from_O);
    }
	
	// FOOTPRINT_from_ARM7
	tf::Transform transform_grasp_FOOTPRINT_from_ARM7 = transformPose(transform_grasp_O_from_SDH, transform_HEADER_from_O, object_pose.header.frame_id);
	//debug
    if (debug)
    {
        geometry_msgs::Transform msg_grasp_FOOTPRINT_from_ARM7;
        tf::transformTFToMsg(transform_grasp_FOOTPRINT_from_ARM7, msg_grasp_FOOTPRINT_from_ARM7);
        ROS_DEBUG_STREAM("msg_grasp_FOOTPRINT_from_ARM7:" << msg_grasp_FOOTPRINT_from_ARM7);
    }
	
	// convert to PoseStamped
	geometry_msgs::Transform msg_transform_grasp_FOOTPRINT_from_ARM7;
	tf::transformTFToMsg(transform_grasp_FOOTPRINT_from_ARM7, msg_transform_grasp_FOOTPRINT_from_ARM7);
	geometry_msgs::PoseStamped msg_pose_grasp_FOOTPRINT_from_ARM7;
	msg_pose_grasp_FOOTPRINT_from_ARM7.header.stamp = ros::Time::now();
	msg_pose_grasp_FOOTPRINT_from_ARM7.header.frame_id = "/base_footprint";
	msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.x = msg_transform_grasp_FOOTPRINT_from_ARM7.translation.x;
	msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.y = msg_transform_grasp_FOOTPRINT_from_ARM7.translation.y;
	msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.z = msg_transform_grasp_FOOTPRINT_from_ARM7.translation.z;
	msg_pose_grasp_FOOTPRINT_from_ARM7.pose.orientation = msg_transform_grasp_FOOTPRINT_from_ARM7.rotation;
    
    if (debug)
    {
        ROS_DEBUG_STREAM("msg_pose_grasp_FOOTPRINT_from_ARM7:" << msg_pose_grasp_FOOTPRINT_from_ARM7);
    }
	
	g.grasp_pose = msg_pose_grasp_FOOTPRINT_from_ARM7;
	
	
	//~~~ ApproachDirection ~~~
	// O_from_SDH
	std::vector<double> current_pre_grasp_pose = current_grasp->GetTCPPreGraspPose();
	tf::Transform transform_pre_O_from_SDH = tf::Transform(
		tf::createQuaternionFromRPY(current_pre_grasp_pose[3], current_pre_grasp_pose[4], current_pre_grasp_pose[5] ),
		0.001*tf::Vector3(current_pre_grasp_pose[0],current_pre_grasp_pose[1],current_pre_grasp_pose[2]));
	//debug
    if (debug)
    {
        geometry_msgs::Transform msg_pre_O_from_SDH;
        tf::transformTFToMsg(transform_pre_O_from_SDH, msg_pre_O_from_SDH);
        ROS_DEBUG_STREAM("msg_pre_O_from_SDH:" << msg_pre_O_from_SDH);
    }
	
	// FOOTPRINT_from_ARM7
	tf::Transform transform_pre_FOOTPRINT_from_ARM7 = transformPose(transform_pre_O_from_SDH, transform_HEADER_from_O, object_pose.header.frame_id);
	//debug
    if (debug)
    {
        geometry_msgs::Transform msg_pre_FOOTPRINT_from_ARM7;
        tf::transformTFToMsg(transform_pre_FOOTPRINT_from_ARM7, msg_pre_FOOTPRINT_from_ARM7);
        ROS_DEBUG_STREAM("msg_pre_FOOTPRINT_from_ARM7:" << msg_pre_FOOTPRINT_from_ARM7);
    }
	
	// convert to PoseStamped
	geometry_msgs::Transform msg_transform_pre_FOOTPRINT_from_ARM7;
	tf::transformTFToMsg(transform_pre_FOOTPRINT_from_ARM7, msg_transform_pre_FOOTPRINT_from_ARM7);
	geometry_msgs::PoseStamped msg_pose_pre_FOOTPRINT_from_ARM7;
	msg_pose_pre_FOOTPRINT_from_ARM7.header.stamp = ros::Time::now();
	msg_pose_pre_FOOTPRINT_from_ARM7.header.frame_id = "/base_footprint";
	msg_pose_pre_FOOTPRINT_from_ARM7.pose.position.x = msg_transform_pre_FOOTPRINT_from_ARM7.translation.x;
	msg_pose_pre_FOOTPRINT_from_ARM7.pose.position.y = msg_transform_pre_FOOTPRINT_from_ARM7.translation.y;
	msg_pose_pre_FOOTPRINT_from_ARM7.pose.position.z = msg_transform_pre_FOOTPRINT_from_ARM7.translation.z;
	msg_pose_pre_FOOTPRINT_from_ARM7.pose.orientation = msg_transform_pre_FOOTPRINT_from_ARM7.rotation;
    if (debug)
    {
        ROS_DEBUG_STREAM("msg_pose_pre_FOOTPRINT_from_ARM7:" << msg_pose_pre_FOOTPRINT_from_ARM7);
    }
	
	g.approach = calculateApproachDirection(msg_pose_grasp_FOOTPRINT_from_ARM7.pose, msg_pose_pre_FOOTPRINT_from_ARM7.pose);
	
	//~~~ RetreatDirection ~~~
	g.retreat.direction.header.frame_id = g.approach.direction.header.frame_id;
	g.retreat.direction.vector.x = -g.approach.direction.vector.x;
	g.retreat.direction.vector.y = -g.approach.direction.vector.y;
	g.retreat.direction.vector.z = -g.approach.direction.vector.z + 0.5; //also lift the object a little bit
	
	g.retreat.min_distance = g.approach.min_distance;
	g.retreat.desired_distance = g.approach.desired_distance;
	
	grasps.push_back(g);
}




void CobPickPlaceActionServer::fillGraspsOR(unsigned int objectClassId, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{
	bool finished_before_timeout;
	grasps.clear();
	
	//ToDo: resolve object_class_name from objectClassId
	if(map_classid_to_classname.find(objectClassId) == map_classid_to_classname.end())
	{
		ROS_ERROR("Unable to resolve class_name for class_id %d", objectClassId);
		return;
	}
	
	
	cob_grasp_generation::QueryGraspsGoal goal_query_grasps;
	goal_query_grasps.object_name = map_classid_to_classname.find(objectClassId)->second;
	goal_query_grasps.grasp_id = grasp_id;
	goal_query_grasps.num_grasps = 0;
	goal_query_grasps.threshold = 0;//0.012;
	
	ac_grasps_or->sendGoal(goal_query_grasps);
	
	ROS_INFO("Querying grasps...");
	finished_before_timeout = ac_grasps_or->waitForResult(ros::Duration(70.0));
	actionlib::SimpleClientGoalState state_grasps_or = ac_grasps_or->getState();
	boost::shared_ptr<const cob_grasp_generation::QueryGraspsResult> result_query_grasps = ac_grasps_or->getResult();
	if (finished_before_timeout)
	{
		ROS_INFO("Action finished: %s",state_grasps_or.toString().c_str());
		
		ROS_INFO("Found %d grasps for this object", result_query_grasps.get()->grasp_list.size());
		for(unsigned int i=0; i<result_query_grasps.get()->grasp_list.size(); i++)
		{
			ROS_DEBUG_STREAM("Grasp "<< i << ": " << result_query_grasps.get()->grasp_list[i]);
		}
		
		for(unsigned int i=0; i<result_query_grasps.get()->grasp_list.size(); i++)
		{
			manipulation_msgs::Grasp current_grasp;
			//~~~ HandGraspConfig ~~~
			current_grasp.grasp_posture = result_query_grasps.get()->grasp_list[i].grasp_posture;
			for(unsigned int k=0; k<current_grasp.grasp_posture.position.size(); k++)
			{
				if(current_grasp.grasp_posture.position[k] < -1.5707) current_grasp.grasp_posture.position[k] = -1.5707;
				if(current_grasp.grasp_posture.position[k] >  1.5707) current_grasp.grasp_posture.position[k] =  1.5707;
			}
			//~~~ HandPreGraspConfig ~~~
			current_grasp.pre_grasp_posture = result_query_grasps.get()->grasp_list[i].pre_grasp_posture;
			
			//~~~ TCPGraspPose ~~~
			///GOAL: -> Get TCPGraspPose for arm_7_link wrt base_footprint
			
			// O_from_SDH
			geometry_msgs::Pose current_grasp_pose = result_query_grasps.get()->grasp_list[i].grasp_pose.pose;
			tf::Transform transform_grasp_O_from_SDH = tf::Transform(
				tf::Quaternion(current_grasp_pose.orientation.x, current_grasp_pose.orientation.y, current_grasp_pose.orientation.z, current_grasp_pose.orientation.w),
				tf::Vector3(current_grasp_pose.position.x, current_grasp_pose.position.y, current_grasp_pose.position.z));
			
			//debug
			geometry_msgs::Transform msg_grasp_O_from_SDH;
			tf::transformTFToMsg(transform_grasp_O_from_SDH, msg_grasp_O_from_SDH);
			ROS_DEBUG_STREAM("msg_grasp_O_from_SDH:" << msg_grasp_O_from_SDH);
			
			// HEADER_from_O (given)
			tf::Transform transform_HEADER_from_O = tf::Transform(
				tf::Quaternion(object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w),
				tf::Vector3(object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z));
			//debug
			geometry_msgs::Transform msg_HEADER_from_O;
			tf::transformTFToMsg(transform_HEADER_from_O, msg_HEADER_from_O);
			ROS_DEBUG_STREAM("msg_HEADER_from_O:" << msg_HEADER_from_O);
			
			// FOOTPRINT_from_ARM7
			tf::Transform transform_grasp_FOOTPRINT_from_ARM7 = transformPose(transform_grasp_O_from_SDH, transform_HEADER_from_O, object_pose.header.frame_id);
			//debug
			geometry_msgs::Transform msg_grasp_FOOTPRINT_from_ARM7;
			tf::transformTFToMsg(transform_grasp_FOOTPRINT_from_ARM7, msg_grasp_FOOTPRINT_from_ARM7);
			ROS_DEBUG_STREAM("msg_grasp_FOOTPRINT_from_ARM7:" << msg_grasp_FOOTPRINT_from_ARM7);
			
			// convert to PoseStamped
			geometry_msgs::Transform msg_transform_grasp_FOOTPRINT_from_ARM7;
			tf::transformTFToMsg(transform_grasp_FOOTPRINT_from_ARM7, msg_transform_grasp_FOOTPRINT_from_ARM7);
			geometry_msgs::PoseStamped msg_pose_grasp_FOOTPRINT_from_ARM7;
			msg_pose_grasp_FOOTPRINT_from_ARM7.header.stamp = ros::Time::now();
			msg_pose_grasp_FOOTPRINT_from_ARM7.header.frame_id = "/base_footprint";
			msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.x = msg_transform_grasp_FOOTPRINT_from_ARM7.translation.x;
			msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.y = msg_transform_grasp_FOOTPRINT_from_ARM7.translation.y;
			msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.z = msg_transform_grasp_FOOTPRINT_from_ARM7.translation.z;
			msg_pose_grasp_FOOTPRINT_from_ARM7.pose.orientation = msg_transform_grasp_FOOTPRINT_from_ARM7.rotation;
			ROS_DEBUG_STREAM("msg_pose_grasp_FOOTPRINT_from_ARM7:" << msg_pose_grasp_FOOTPRINT_from_ARM7);
			
			current_grasp.grasp_pose = msg_pose_grasp_FOOTPRINT_from_ARM7;
			
			//~~~ ApproachDirection ~~~
			//current_grasp.approach.direction.header.frame_id = "/base_footprint";
			//current_grasp.approach.direction.vector.x = 0.0;
			//current_grasp.approach.direction.vector.y = 0.0;
			//current_grasp.approach.direction.vector.z = -1.0;
			//current_grasp.approach.min_distance = 0.18;
			//current_grasp.approach.desired_distance = 0.28;
			
			current_grasp.approach.direction.header.frame_id = "/arm_7_link";
			current_grasp.approach.direction.vector.x = 0.0;
			current_grasp.approach.direction.vector.y = 0.0;
			current_grasp.approach.direction.vector.z = 1.0;
			current_grasp.approach.min_distance = 0.18;
			current_grasp.approach.desired_distance = 0.28;
			
			//current_grasp.approach.direction.header.frame_id = "/base_footprint";
			//current_grasp.approach.direction.vector.x = -msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.x+object_pose.pose.position.x;
			//current_grasp.approach.direction.vector.y = -msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.y+object_pose.pose.position.y;
			//current_grasp.approach.direction.vector.z = -msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.z+object_pose.pose.position.z;
			//current_grasp.approach.min_distance = 0.18;
			//current_grasp.approach.desired_distance = 0.28;
			
			//~~~ RetreatDirection ~~~
			current_grasp.retreat.direction.header.frame_id = "/base_footprint";
			current_grasp.retreat.direction.vector.x = 0.0;
			current_grasp.retreat.direction.vector.y = 0.0;
			current_grasp.retreat.direction.vector.z = 1.0;
			current_grasp.retreat.min_distance = 0.1;
			current_grasp.retreat.desired_distance = 0.15;
			
			current_grasp.retreat.direction.header.frame_id = "/arm_7_link";
			current_grasp.retreat.direction.vector.x = 0.0;
			current_grasp.retreat.direction.vector.y = 0.0;
			current_grasp.retreat.direction.vector.z = -1.0;
			current_grasp.retreat.min_distance = 0.1;
			current_grasp.retreat.desired_distance = 0.15;
			
			grasps.push_back(current_grasp);
		}
		
	}
	else
		ROS_ERROR("Grasps not queried within timeout");
}






sensor_msgs::JointState CobPickPlaceActionServer::MapHandConfiguration(sensor_msgs::JointState table_config)
{
	sensor_msgs::JointState grasp_configuration;
	grasp_configuration.position.resize(7);
	grasp_configuration.position[0]= table_config.position[0];
	grasp_configuration.position[1]= table_config.position[2];
	grasp_configuration.position[2]= table_config.position[3];
	grasp_configuration.position[3]= table_config.position[5];
	grasp_configuration.position[4]= table_config.position[6];
	grasp_configuration.position[5]= table_config.position[11];
	grasp_configuration.position[6]= table_config.position[12];
	
	grasp_configuration.name.resize(7);
	grasp_configuration.name[0]=("sdh_knuckle_joint");
	grasp_configuration.name[1]=("sdh_thumb_2_joint");
	grasp_configuration.name[2]=("sdh_thumb_3_joint");
	grasp_configuration.name[3]=("sdh_finger_12_joint");
	grasp_configuration.name[4]=("sdh_finger_13_joint");
	grasp_configuration.name[5]=("sdh_finger_22_joint");
	grasp_configuration.name[6]=("sdh_finger_23_joint");    

	//cut joint_values according to joint_limits
	for(unsigned int i=0; i<grasp_configuration.position.size();i++)
	{
		if(grasp_configuration.position[i]>1.57079){grasp_configuration.position[i]=1.57079;}
		if(grasp_configuration.position[i]<-1.57079){grasp_configuration.position[i]=-1.57079;}
	}
	return grasp_configuration;
}

tf::Transform CobPickPlaceActionServer::transformPose(tf::Transform transform_O_from_SDH, tf::Transform transform_HEADER_from_O, std::string object_frame_id)
{
    bool debug = false;
	// SDH_from_ARM7
	tf::StampedTransform transform_SDH_from_ARM7;
	try
	{
		ros::Time now = ros::Time::now();
		tf_listener_.waitForTransform("/sdh_palm_link", "/arm_7_link", now, ros::Duration(10.0));
		tf_listener_.lookupTransform("/sdh_palm_link", "/arm_7_link", now, transform_SDH_from_ARM7);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	//debug
	geometry_msgs::TransformStamped msg_SDH_from_ARM7;
	tf::transformStampedTFToMsg(transform_SDH_from_ARM7, msg_SDH_from_ARM7);
    
    if (debug)
        ROS_DEBUG_STREAM("msg_SDH_from_ARM7:" << msg_SDH_from_ARM7);
	
	// O_from_ARM7 = O_from_SDH * SDH_from_ARM7
	tf::Transform transform_O_from_ARM7 = transform_O_from_SDH * transform_SDH_from_ARM7;
	
	// FOOTPRINT_from_HEADER
	tf::StampedTransform transform_FOOTPRINT_from_HEADER;
	try
	{
		ros::Time now = ros::Time::now();
		tf_listener_.waitForTransform("/base_footprint", object_frame_id, now, ros::Duration(10.0));
		tf_listener_.lookupTransform("/base_footprint", object_frame_id, now, transform_FOOTPRINT_from_HEADER);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	//debug
	geometry_msgs::TransformStamped msg_FOOTPRINT_from_HEADER;
	tf::transformStampedTFToMsg(transform_FOOTPRINT_from_HEADER, msg_FOOTPRINT_from_HEADER);
    
    if (debug)
        ROS_DEBUG_STREAM("msg_FOOTPRINT_from_HEADER:" << msg_FOOTPRINT_from_HEADER);
	
	// FOOTPRINT_from_O = FOOTPRINT_from_HEADER * HEADER_from_O
	tf::Transform transform_FOOTPRINT_from_O = transform_FOOTPRINT_from_HEADER * transform_HEADER_from_O;
	
	// FOOTPRINT_from_ARM7 = FOOTPRINT_from_O * O_from_ARM7
	tf::Transform transform_FOOTPRINT_from_ARM7 = transform_FOOTPRINT_from_O * transform_O_from_ARM7;
	
	return transform_FOOTPRINT_from_ARM7;
}

manipulation_msgs::GripperTranslation CobPickPlaceActionServer::calculateApproachDirection(geometry_msgs::Pose msg_pose_grasp_FOOTPRINT_from_ARM7, geometry_msgs::Pose msg_pose_pre_FOOTPRINT_from_ARM7)
{
	double finger_length = 0.18; //this is the lenght of the sdh ('home' configuration)
	manipulation_msgs::GripperTranslation approach;
	approach.direction.header.frame_id = "/base_footprint";
	//~ dis=sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2)
	//~ direction.x= (x1-x0)/dis and likewise
	
	double distance = sqrt(
		pow((msg_pose_grasp_FOOTPRINT_from_ARM7.position.x-msg_pose_pre_FOOTPRINT_from_ARM7.position.x),2) +
		pow((msg_pose_grasp_FOOTPRINT_from_ARM7.position.y-msg_pose_pre_FOOTPRINT_from_ARM7.position.y),2) +
		pow((msg_pose_grasp_FOOTPRINT_from_ARM7.position.z-msg_pose_pre_FOOTPRINT_from_ARM7.position.z),2));
	
	ROS_DEBUG("Distance between pre-grasp and grasp: %f", distance);
	
	approach.direction.vector.x = (msg_pose_grasp_FOOTPRINT_from_ARM7.position.x-msg_pose_pre_FOOTPRINT_from_ARM7.position.x)/distance;
	approach.direction.vector.y = (msg_pose_grasp_FOOTPRINT_from_ARM7.position.y-msg_pose_pre_FOOTPRINT_from_ARM7.position.y)/distance;
	approach.direction.vector.z = (msg_pose_grasp_FOOTPRINT_from_ARM7.position.z-msg_pose_pre_FOOTPRINT_from_ARM7.position.z)/distance;
	
	//min_distance means that we want to follow at least this length along the apporach_vector
	if(distance < finger_length)
	{
		approach.min_distance = finger_length;
	}
	else
	{
		approach.min_distance = distance;
	}
	approach.desired_distance = approach.min_distance + 0.1;
	
	return approach;
}















int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_pick_place_action_server_node");
	CobPickPlaceActionServer *cob_pick_place_action_server = new CobPickPlaceActionServer();
	
	cob_pick_place_action_server->initialize();
	cob_pick_place_action_server->run();
	
	return 0;
}
