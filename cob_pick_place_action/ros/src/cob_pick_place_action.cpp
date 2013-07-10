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

	static const std::string COB_COLLISION_OBJECT_NAME = "cob_collision_object_action";
	as_collision_object.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobCollisionObjectAction>(nh_, COB_COLLISION_OBJECT_NAME, boost::bind(&CobPickPlaceActionServer::collision_object_goal_cb, this, _1), false));
	as_collision_object->start();

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
		
<<<<<<< Updated upstream
	//setupEnvironment();
=======
	setupEnvironment();
>>>>>>> Stashed changes
}

void CobPickPlaceActionServer::run()
{
	ROS_INFO("cob_pick_action...spinning");
    ros::spin();
}

void CobPickPlaceActionServer::collision_object_goal_cb(const cob_pick_place_action::CobCollisionObjectGoalConstPtr &goal)
{
	ROS_INFO("received collision object %s", goal->object.id.c_str());
	bool success = false;
	

	pub_co.publish(goal->object);
	ros::Duration(1.0).sleep();

	// Special handling of keyword 'support_surface'
	if (goal->object.id == "support_surface" && goal->object.operation != goal->object.REMOVE)
	{
		ROS_INFO("Setting support surface");
		group.setSupportSurfaceName("support_surface");
	}
	else if (goal->object.id == "support_surface" && goal->object.operation == goal->object.REMOVE)
	{
		ROS_INFO("Removing support surface");
		group.setSupportSurfaceName("");
	}
	
	// Special handling of keyword 'all'
	if (goal->object.id == "all" && goal->object.operation == goal->object.REMOVE)
	{
		ROS_INFO("Detaching and removing all collision models");
		resetEnvironment();
	}
	
	// Setting result
	success = true;
	cob_pick_place_action::CobCollisionObjectResult result;
	if(success)
	{
		result.success.data=true;
                as_collision_object->setSucceeded(result);
	}
	else
	{
		result.success.data=false;
                as_collision_object->setAborted(result);
	}
}

void CobPickPlaceActionServer::pick_goal_cb(const cob_pick_place_action::CobPickGoalConstPtr &goal)
{
	ROS_INFO("PickGoalCB: Received new goal: Trying to pick %s", goal->object_name.c_str());
	bool success = false;
	
	ROS_DEBUG_STREAM(*(goal.get()));

	///Updating the object collision_object
	insertObject(goal->object_name, goal->object_pose);
	
	///Get grasps from corresponding GraspTable
	std::vector<manipulation_msgs::Grasp> grasps;
	fillAllGrasps(goal->object_id, goal->object_pose, grasps);
	//unsigned int grasp_id = goal->grasp_id;
	//fillSingleGrasp(goal->object_id, grasp_id, goal->object_pose, grasps);
	
	ROS_INFO("PickGoalCB: Found %d grasps for this object", grasps.size());
	for(unsigned int i=0; i<grasps.size(); i++)
	{
		ROS_DEBUG_STREAM("Grasp "<< i << ": " << grasps[i]);
	}
	
<<<<<<< Updated upstream
	if(!(goal->support_surface.empty()))
	{
		ROS_INFO("Setting SupportSurface to %s", goal->support_surface.c_str());
		group.setSupportSurfaceName(goal->support_surface);
	}
=======
	
	//group.setSupportSurfaceName("table");
>>>>>>> Stashed changes
	group.setPlanningTime(60.0);	//default is 5.0 s
	
	
	///Call Pick with result
	manipulation_msgs::Grasp grasp_used;
	success = group.pick(goal->object_name, grasps, grasp_used);
	ROS_DEBUG_STREAM("Executed Grasp: " << grasp_used);
	
	
	///Setting result
	cob_pick_place_action::CobPickResult result;
	std::string response;
	if(success)
	{
		result.success.data=true;
		response="Hurray!!! Pickup COMPLETE";
		as_pick->setSucceeded(result, response);
		last_grasp_valid = true;
		last_object_name = goal->object_name;
		last_grasp = grasp_used;
	}
	else
	{
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
	
	std::vector<manipulation_msgs::PlaceLocation> locations;
	manipulation_msgs::PlaceLocation place_location;
	
	place_location.id = "Last_"+goal->object_name+"_grasp";
	place_location.post_place_posture = last_grasp.pre_grasp_posture;
	place_location.place_pose = goal->destination;
	place_location.approach.direction.header.frame_id = "/base_footprint";
	place_location.approach.direction.vector.z = -1.0;
	place_location.approach.min_distance = 0.1;
	place_location.approach.desired_distance = 0.2;
	place_location.retreat.direction.header.frame_id = "/base_footprint";
	place_location.retreat.direction.vector.z = 1.0;
	place_location.retreat.min_distance = 0.1;
	place_location.retreat.desired_distance = 0.25;
	
	locations.push_back(place_location);

	//group.setSupportSurfaceName("table");
	group.setPlanningTime(60.0);	//default is 5.0 s
	
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




void CobPickPlaceActionServer::setupEnvironment()
{
	ros::Duration(1.0).sleep();
}

void CobPickPlaceActionServer::resetEnvironment()
{
	ros::Duration(1.0).sleep();
	
	ROS_INFO("Detaching all objects");
	moveit_msgs::AttachedCollisionObject object;
	//object.object.id = "";
	object.link_name = "arm_7_link";
	object.object.operation = object.object.REMOVE;
	pub_ao.publish(object);
	
	ROS_INFO("Removing all objects");
	moveit_msgs::CollisionObject co;
	//co.id = object_name;
	co.operation = co.REMOVE;
	pub_co.publish(co);
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
	
	//// add object as SolidPrimitive
	//co.id = object_name;
	//co.operation = co.ADD;
	//co.primitives.resize(1);
	////sauerkraut
	//co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	//co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
	//co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.12;
	//co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.05;
	//////fruitdrink
	////co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	////co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	////co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.0768;
	////co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.0824;
	////co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1484;
	//co.primitive_poses.resize(1);
	//co.primitive_poses[0] = object_pose.pose;
	//pub_co.publish(co);
	
	
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

void CobPickPlaceActionServer::detachObject(std::string object_name)
{
	moveit_msgs::AttachedCollisionObject object;
	object.object.id = object_name;
	object.link_name = "arm_7_link";
	object.object.operation = object.object.REMOVE;
	pub_ao.publish(object);
	
	ros::Duration(1.0).sleep();
	
	moveit_msgs::CollisionObject co;
	
	// remove object
	co.id = object_name;
	co.operation = co.REMOVE;
	pub_co.publish(co);
	
	ros::Duration(1.0).sleep();
}






void CobPickPlaceActionServer::fillAllGrasps(unsigned int objectClassId, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{  
	grasps.clear(); 
	manipulation_msgs::Grasp g;
	
	m_GraspTable->ResetReadPtr(objectClassId);
	Grasp *current_grasp = NULL;
	unsigned int grasp_index = 0;
	tf::TransformListener listener;
	tf::StampedTransform transform_object_frameid_footprint;
	try
	{
		listener.lookupTransform("/base_footprint", object_pose.header.frame_id, ros::Time::now(), transform_object_frameid_footprint);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	
	current_grasp = m_GraspTable->GetNextGrasp(objectClassId);
	
	while(current_grasp)
	{
		ROS_DEBUG("GraspIndex: %d",grasp_index);
		
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
		
		//TCPGraspPose
		std::vector<double> current_grasp_pose = current_grasp->GetTCPGraspPose();
		geometry_msgs::Pose grasp_pose_wrt_object;
		grasp_pose_wrt_object.position.x=current_grasp_pose[0];
		grasp_pose_wrt_object.position.y=current_grasp_pose[1];
		grasp_pose_wrt_object.position.z=current_grasp_pose[2];
		grasp_pose_wrt_object.orientation=tf::createQuaternionMsgFromRollPitchYaw(current_grasp_pose[3], current_grasp_pose[4], current_grasp_pose[5] );
		
		///TODO: VERIFY THIS!!
		//Get grasp_pose in base_footprint
		geometry_msgs::PoseStamped grasp_pose_wrt_footprint = transformPose(grasp_pose_wrt_object, object_pose, transform_object_frameid_footprint);
		g.grasp_pose= grasp_pose_wrt_footprint;
		
		//ApproachDirection
		std::vector<double> current_pre_grasp_pose = current_grasp->GetTCPPreGraspPose();
		geometry_msgs::Pose pre_grasp_pose_wrt_object;
		pre_grasp_pose_wrt_object.position.x=current_pre_grasp_pose[0];
		pre_grasp_pose_wrt_object.position.y=current_pre_grasp_pose[1];
		pre_grasp_pose_wrt_object.position.z=current_pre_grasp_pose[2];
		pre_grasp_pose_wrt_object.orientation=tf::createQuaternionMsgFromRollPitchYaw(current_pre_grasp_pose[3], current_pre_grasp_pose[4], current_pre_grasp_pose[5] );
		
		///TODO: VERIFY THIS!!
		//Get grasp_pose in base_footprint
		geometry_msgs::PoseStamped  pre_grasp_pose_wrt_footprint = transformPose(pre_grasp_pose_wrt_object, object_pose, transform_object_frameid_footprint);
		
		g.approach = calculateApproachDirection(grasp_pose_wrt_footprint.pose, pre_grasp_pose_wrt_footprint.pose);
		
		//RetreatDirection
		g.retreat.direction.header.frame_id = "/base_footprint";
		g.retreat.direction.vector.z = 1.0;
		g.retreat.min_distance = 0.1;
		g.retreat.desired_distance = 0.25;
		
		grasps.push_back(g);
		current_grasp = m_GraspTable->GetNextGrasp(objectClassId);
		
		grasp_index++;
	}
}

void CobPickPlaceActionServer::fillSingleGrasp(unsigned int objectClassId, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{
	grasps.clear(); 
	manipulation_msgs::Grasp g;
	tf::TransformListener listener;
	tf::StampedTransform transform_object_frameid_footprint;
	try
	{
		listener.lookupTransform("/base_footprint", object_pose.header.frame_id, ros::Time::now(), transform_object_frameid_footprint);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	Grasp *current_grasp = NULL;
	
	current_grasp = m_GraspTable->GetGrasp(objectClassId, grasp_id);
	
	if(current_grasp)
	{
		ROS_INFO("GraspIndex %d found",grasp_id);
		
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
		std::vector<double> current_hand_config = current_grasp->GetHandGraspConfig();
		sensor_msgs::JointState grasp_posture;
		grasp_posture.position.clear();
		for (unsigned int i=0; i<current_hand_config.size(); i++)
		{
			grasp_posture.position.push_back(current_hand_config[i]);
		}
		g.grasp_posture= MapHandConfiguration(grasp_posture);
		
		//TCPGraspPose
		std::vector<double> current_grasp_pose = current_grasp->GetTCPGraspPose();
		geometry_msgs::Pose grasp_pose_wrt_object;
		grasp_pose_wrt_object.position.x=current_grasp_pose[0];
		grasp_pose_wrt_object.position.y=current_grasp_pose[1];
		grasp_pose_wrt_object.position.z=current_grasp_pose[2];
		grasp_pose_wrt_object.orientation=tf::createQuaternionMsgFromRollPitchYaw(current_grasp_pose[3], current_grasp_pose[4], current_grasp_pose[5] );
		
		///TODO: VERIFY THIS!!
		//Get grasp_pose in base_footprint
		geometry_msgs::PoseStamped grasp_pose_wrt_footprint = transformPose(grasp_pose_wrt_object, object_pose, transform_object_frameid_footprint);
		g.grasp_pose= grasp_pose_wrt_footprint;
		
		//ApproachDirection
		std::vector<double> current_pre_grasp_pose = current_grasp->GetTCPPreGraspPose();
		geometry_msgs::Pose pre_grasp_pose_wrt_object;
		pre_grasp_pose_wrt_object.position.x=current_pre_grasp_pose[0];
		pre_grasp_pose_wrt_object.position.y=current_pre_grasp_pose[1];
		pre_grasp_pose_wrt_object.position.z=current_pre_grasp_pose[2];
		pre_grasp_pose_wrt_object.orientation=tf::createQuaternionMsgFromRollPitchYaw(current_pre_grasp_pose[3], current_pre_grasp_pose[4], current_pre_grasp_pose[5] );
		
		///TODO: VERIFY THIS!!
		//Get grasp_pose in base_footprint
		geometry_msgs::PoseStamped  pre_grasp_pose_wrt_footprint = transformPose(pre_grasp_pose_wrt_object, object_pose, transform_object_frameid_footprint);
		
		g.approach = calculateApproachDirection(grasp_pose_wrt_footprint.pose, pre_grasp_pose_wrt_footprint.pose);
		
		
		//RetreatDirection
		g.retreat.direction.header.frame_id = "base_footprint";
		g.retreat.direction.vector.z = 1.0;
		g.retreat.min_distance = 0.1;
		g.retreat.desired_distance = 0.25;
		
		grasps.push_back(g);
	}
	else
		ROS_ERROR("Grasp %d NOT found", grasp_id);
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

geometry_msgs::PoseStamped CobPickPlaceActionServer::transformPose(geometry_msgs::Pose grasp_pose_wrt_object, geometry_msgs::PoseStamped object_pose, tf::StampedTransform transform_object_frameid_footprint)
{
	//~ Getting transform for object wrt its header frame
	tf::Quaternion quat;
	tf::quaternionMsgToTF(object_pose.pose.orientation, quat);
	tf::Vector3 vec = tf::Vector3(object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);
	tf::Transform trans_obj_wrt_header = tf::Transform(quat, vec);
	
	//~ Getting transform for grasp wrt object
	tf::quaternionMsgToTF(grasp_pose_wrt_object.orientation, quat);
	vec = 0.001*tf::Vector3(grasp_pose_wrt_object.position.x, grasp_pose_wrt_object.position.y, grasp_pose_wrt_object.position.z);
	vec = vec + tf::Vector3(0,0,-0.032);	//transform from sdh_palm_link to arm_7_link
	tf::Transform trans_grasp_obj = tf::Transform(quat, vec);
	
	//~ Getting transform for grasp wrt footprint
	tf::Transform trans_grasp_wrt_object_header = trans_obj_wrt_header.operator*(trans_grasp_obj);
	tf::Transform trans_grasp_wrt_footprint = transform_object_frameid_footprint.operator*(trans_grasp_wrt_object_header);

	geometry_msgs::Transform msg;
	tf::transformTFToMsg(trans_grasp_wrt_footprint, msg);
	
	geometry_msgs::PoseStamped grasp_pose_wrt_base_footprint;
	grasp_pose_wrt_base_footprint.header.frame_id=object_pose.header.frame_id;
	grasp_pose_wrt_base_footprint.pose.position.x = msg.translation.x;
	grasp_pose_wrt_base_footprint.pose.position.y = msg.translation.y;
	grasp_pose_wrt_base_footprint.pose.position.z = msg.translation.z;
	grasp_pose_wrt_base_footprint.pose.orientation = msg.rotation;
	return grasp_pose_wrt_base_footprint;
}

manipulation_msgs::GripperTranslation CobPickPlaceActionServer::calculateApproachDirection(geometry_msgs::Pose grasp_pose_wrt_footprint, geometry_msgs::Pose pre_grasp_pose_wrt_footprint)
{
	manipulation_msgs::GripperTranslation approach;
	approach.direction.header.frame_id = "/base_footprint";
	//~ dis=sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2)
	//~ direction.x= (x1-x0)/dis and likewise
	
	double distance = sqrt(pow((grasp_pose_wrt_footprint.position.x-pre_grasp_pose_wrt_footprint.position.x),2)+pow((grasp_pose_wrt_footprint.position.y-pre_grasp_pose_wrt_footprint.position.y),2)+pow((grasp_pose_wrt_footprint.position.z-pre_grasp_pose_wrt_footprint.position.z),2));
	
	approach.direction.vector.x = (grasp_pose_wrt_footprint.position.x-pre_grasp_pose_wrt_footprint.position.x)/distance;
	approach.direction.vector.y = (grasp_pose_wrt_footprint.position.y-pre_grasp_pose_wrt_footprint.position.y)/distance;
	approach.direction.vector.z = (grasp_pose_wrt_footprint.position.z-pre_grasp_pose_wrt_footprint.position.z)/distance;
	
	approach.min_distance = distance;
	ROS_DEBUG("Distance between pre-grasp and grasp: %f", distance);
	approach.desired_distance = approach.min_distance + 0.2;
	
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
