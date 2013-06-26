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
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <cob_pick_place_action/cob_pick_place_action.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"


void CobPickPlaceActionServer::initialize()
{
	pub_co = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	pub_ao = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	
	static const std::string COB_PICKUP_ACTION_NAME = "cob_pick_action";
	as_pick.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobPickAction>(nh_, COB_PICKUP_ACTION_NAME, boost::bind(&CobPickPlaceActionServer::pick_goal_cb, this, _1), false));
	as_pick->start();
	
	static const std::string COB_PLACE_ACTION_NAME = "cob_place_action";
	as_place.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobPlaceAction>(nh_, COB_PLACE_ACTION_NAME, boost::bind(&CobPickPlaceActionServer::place_goal_cb, this, _1), false));
	as_place->start();
	
	//~ Grasp Table Initializations################################
	std::string path = ros::package::getPath("cob_pick_place_action")+std::string("/files/GraspTable.txt");
	GraspTableIniFile = const_cast<char*>(path.c_str());  
	m_GraspTable = new GraspTable();
	int error = m_GraspTable->Init(GraspTableIniFile);
	
	if(error<0)
		ROS_ERROR("Failed to initialize GraspTables");
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
	
	///Setting up the environment
	//TODO: This is done somewhere else later. Only update position of grasp_object
	setUpEnvironment(goal->object_name, goal->object_pose);
	
	
	///Get grasps from corresponding GraspTable
	std::vector<manipulation_msgs::Grasp> grasps;
	//fillAllGrasps(goal->object_id, goal->object_pose, grasps);
	unsigned int grasp_id = 23;
	fillSingleGrasp(goal->object_id, grasp_id, goal->object_pose, grasps);
	
	ROS_INFO("PickGoalCB: Found %d grasps for this object", grasps.size());
	for(unsigned int i=0; i<grasps.size(); i++)
	{
		ROS_DEBUG_STREAM("Grasp "<< i << ": " << grasps[i]);
	}
	
	group.setSupportSurfaceName("table");
	
	
	success = group.pick(goal->object_name, grasps);
	
	
	///Setting result
	cob_pick_place_action::CobPickResult result;
	std::string response;
	if(success)
	{
		result.success.data=true;
		response="Hurray!!! Pickup COMPLETE";
		as_pick->setSucceeded(result, response);
	}
	else
	{
		result.success.data=false;
		response="Alas!!! Pickup FAILED";
		as_pick->setAborted(result, response);
	}
}

void CobPickPlaceActionServer::place_goal_cb(const cob_pick_place_action::CobPlaceGoalConstPtr &goal)
{
	ROS_INFO("PlaceGoalCB: Received new goal");
	bool success = false;
	
	///reslease object
	detachObject(goal->object_name);
	success = true;
	
	///Setting result
	cob_pick_place_action::CobPlaceResult result;
	std::string response;
	if(success)
	{
		result.success.data=true;
		response="PLACE SUCCEEDED";
		as_place->setSucceeded(result, response);
	}
	else
	{
		result.success.data=false;
		response="PLACE FAILED";
		as_place->setAborted(result, response);
	}
}




void CobPickPlaceActionServer::setUpEnvironment(std::string object_name, geometry_msgs::PoseStamped object_pose)
{
	ROS_INFO("Setting up environment..");
	
	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_footprint";
	
	// remove pole
	co.id = "pole";
	co.operation = co.REMOVE;
	pub_co.publish(co);

	// add pole
	co.id = "pole";
	co.operation = co.ADD;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.7;
	co.primitive_poses[0].position.y = -0.4;  
	co.primitive_poses[0].position.z = 0.85;
	co.primitive_poses[0].orientation.w = 1.0;
	pub_co.publish(co);

	// remove table
	co.id = "table";
	co.operation = co.REMOVE;
	pub_co.publish(co);

	// add table
	co.id = "table";
	co.operation = co.ADD;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
	co.primitive_poses[0].position.x = 0.7;
	co.primitive_poses[0].position.y = -0.2;  
	co.primitive_poses[0].position.z = 0.175;
	pub_co.publish(co);
	
	
	
	// remove object
	co.header.frame_id = object_pose.header.frame_id;
	co.id = object_name;
	co.operation = co.REMOVE;
	pub_co.publish(co);
	
	// add object
	co.header.frame_id = object_pose.header.frame_id;
	co.id = object_name;
	co.operation = co.ADD;
	
	//sauerkraut
	co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.12;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.05;
	////fruitdrink
	//co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	//co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.0768;
	//co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.0824;
	//co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1484;
	
	co.primitive_poses[0] = object_pose.pose;
	pub_co.publish(co);
	
	//// add object
	//co = moveit_msgs::CollisionObject();
	//co.header.stamp = ros::Time::now();
	//co.header.frame_id = "base_footprint";
	//co.id = object_name;
	//co.operation = co.ADD;
	//boost::scoped_ptr<shapes::Mesh> mesh;
	//mesh.reset(shapes::createMeshFromResource("package://cob_pick_place_action/files/meshes/sauerkraut.stl"));
	////mesh.reset(shapes::createMeshFromResource("package://cob_pick_place_action/files/meshes/fruitdrink.stl"));
	//mesh->scale(0.001);
	//shapes::ShapeMsg shape_msg;
	//shapes::constructMsgFromShape(mesh.get(), shape_msg);    
	//co.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
	//geometry_msgs::Pose mesh_pose;
	//mesh_pose.position.x = -0.5;
	//mesh_pose.position.y = -0.5;  
	//mesh_pose.position.z = 0.6;
	//co.mesh_poses.push_back(mesh_pose);
	//pub_co.publish(co);
	
	ros::Duration(1.0).sleep();
}

void CobPickPlaceActionServer::detachObject(std::string object_name)
{
	moveit_msgs::AttachedCollisionObject object;
	object.object.id = object_name;
	object.link_name = "arm_7_link";
	object.object.operation = object.object.REMOVE;
	pub_ao.publish(object);
}






void CobPickPlaceActionServer::fillAllGrasps(unsigned int objectClassId, geometry_msgs::PoseStamped object_pose, std::vector<manipulation_msgs::Grasp> &grasps)
{  
	grasps.clear(); 
	manipulation_msgs::Grasp g;
	
	m_GraspTable->ResetReadPtr(objectClassId);
	Grasp *current_grasp = NULL;
	unsigned int grasp_index = 0;
	
	current_grasp = m_GraspTable->GetNextGrasp(objectClassId);
	
	while(current_grasp)
	{
		ROS_INFO("GraspIndex: %d",grasp_index);
		
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
		//Get grasp_pose in Base_footprint
		g.grasp_pose.pose = transformPose(grasp_pose_wrt_object, object_pose.pose);
		g.grasp_pose.header.frame_id = "base_footprint";
		
		//ApproachDirection
		std::vector<double> current_pre_grasp_pose = current_grasp->GetTCPPreGraspPose();
		g.approach = calculateApproachDirection(current_grasp_pose, current_pre_grasp_pose);
		
		//RetreatDirection
		g.retreat.direction.header.frame_id = "base_footprint";
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
		//Get grasp_pose in Base_footprint
		g.grasp_pose.pose = transformPose(grasp_pose_wrt_object, object_pose.pose);
		g.grasp_pose.header.frame_id = "base_footprint";
		
		//ApproachDirection
		std::vector<double> current_pre_grasp_pose = current_grasp->GetTCPPreGraspPose();
		g.approach = calculateApproachDirection(current_grasp_pose, current_pre_grasp_pose);
		
		//RetreatDirection
		g.retreat.direction.header.frame_id = "base_footprint";
		g.retreat.direction.vector.z = 1.0;
		g.retreat.min_distance = 0.1;
		g.retreat.desired_distance = 0.25;
		
		grasps.push_back(g);
		//grasps.push_back(g);
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

geometry_msgs::Pose CobPickPlaceActionServer::transformPose(geometry_msgs::Pose grasp_pose_wrt_object, geometry_msgs::Pose object_pose)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(object_pose.orientation, quat);
	tf::Vector3 vec = tf::Vector3(object_pose.position.x, object_pose.position.y, object_pose.position.z);
	tf::Transform trans_obj_footprint = tf::Transform(quat, vec);
	
	tf::quaternionMsgToTF(grasp_pose_wrt_object.orientation, quat);
	vec = 0.001*tf::Vector3(grasp_pose_wrt_object.position.x, grasp_pose_wrt_object.position.y, grasp_pose_wrt_object.position.z);
	tf::Transform trans_grasp_obj = tf::Transform(quat, vec);
	
	tf::Transform trans_grasp_footprint = trans_obj_footprint.operator*(trans_grasp_obj);
	
	geometry_msgs::Transform msg;
	tf::transformTFToMsg(trans_grasp_footprint, msg);
	
	geometry_msgs::Pose grasp_pose_wrt_footprint;
	grasp_pose_wrt_footprint.position.x = msg.translation.x;
	grasp_pose_wrt_footprint.position.y = msg.translation.y;
	grasp_pose_wrt_footprint.position.z = msg.translation.z;
	grasp_pose_wrt_footprint.orientation = msg.rotation;
	
	return grasp_pose_wrt_footprint;
}

manipulation_msgs::GripperTranslation CobPickPlaceActionServer::calculateApproachDirection(std::vector<double> current_grasp_pose, std::vector<double> current_pre_grasp_pose)
{
	manipulation_msgs::GripperTranslation approach;
	approach.direction.header.frame_id = "sdh_palm_link";
	//~ dis=sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2)
	//~ direction.x= (x1-x0)/dis and likewise
	approach.desired_distance=sqrt(pow((current_grasp_pose[0]-current_pre_grasp_pose[0]),2)+pow((current_grasp_pose[1]-current_pre_grasp_pose[1]),2)+pow((current_grasp_pose[2]-current_pre_grasp_pose[2]),2));
	approach.direction.vector.x = (current_grasp_pose[0]-current_pre_grasp_pose[0])/approach.desired_distance;
	approach.direction.vector.y = (current_grasp_pose[1]-current_pre_grasp_pose[1])/approach.desired_distance;
	approach.direction.vector.z = (current_grasp_pose[2]-current_pre_grasp_pose[2])/approach.desired_distance;
	approach.min_distance = 0.2;
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
