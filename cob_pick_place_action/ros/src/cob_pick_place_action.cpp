/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
 
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <cob_pick_place_action/cob_pick_place_action.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

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

	//non-KIT objects
	map_classid_to_classname[5001]="pringles";


	static const std::string COB_PICKUP_ACTION_NAME = "cob_pick_action";
	as_pick.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobPickAction>(nh_, COB_PICKUP_ACTION_NAME, boost::bind(&CobPickPlaceActionServer::pick_goal_cb, this, _1), false));
	as_pick->start();

	static const std::string COB_PLACE_ACTION_NAME = "cob_place_action";
	as_place.reset(new actionlib::SimpleActionServer<cob_pick_place_action::CobPlaceAction>(nh_, COB_PLACE_ACTION_NAME, boost::bind(&CobPickPlaceActionServer::place_goal_cb, this, _1), false));
	as_place->start();

	last_grasp_valid = false;
	last_object_name.clear();


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
	cob_pick_place_action::CobPickResult result;
	std::string response;
	bool success = false;

	ROS_DEBUG_STREAM(*(goal.get()));

	if(goal->gripper_type.empty())
	{
		ROS_ERROR("Pick failed: No gripper_type specified!");
		result.success.data=false;
		response="Pick failed: No gripper_type specified!";
		as_pick->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
		return;
	}

	///Get grasps from corresponding GraspTable
	std::vector<moveit_msgs::Grasp> grasps;
	if(goal->grasp_database=="KIT")
	{
		ROS_INFO("Using KIT grasp table");
		if(goal->grasp_id!=0)
		{
			ROS_INFO("Using specific grasp_id: %d", goal->grasp_id);
			fillSingleGraspKIT(goal->object_class, goal->gripper_type, goal->grasp_id, goal->object_pose, grasps);
		}
		else
		{
			ROS_INFO("Using all grasps");
			fillAllGraspsKIT(goal->object_class, goal->gripper_type, goal->object_pose, grasps);
		}
	}
	else if(goal->grasp_database=="OpenRAVE")
	{
		ROS_INFO("Using OpenRAVE grasp table");
		fillGraspsOR(goal->object_class, goal->gripper_type, goal->gripper_side, goal->grasp_id, goal->object_pose, grasps);
	}
	else if(goal->grasp_database=="ALL")
	{
		ROS_INFO("Using all available databases");
		std::vector<moveit_msgs::Grasp> grasps_OR, grasps_KIT;
		fillAllGraspsKIT(goal->object_class, goal->gripper_type, goal->object_pose, grasps_KIT);
		fillGraspsOR(goal->object_class, goal->gripper_type, goal->gripper_side, goal->grasp_id, goal->object_pose, grasps_OR);

		grasps = grasps_KIT;
		std::vector<moveit_msgs::Grasp>::iterator it = grasps.end();
		grasps.insert(it, grasps_OR.begin(), grasps_OR.end());
	}
	else
	{
		ROS_ERROR("Grasp_Database %s not supported! Please use \"KIT\" or \"OpenRAVE\" or \"ALL\" instead", goal->grasp_database.c_str());
		result.success.data=false;
		response="Pick failed: Grasp Database not supported!";
		as_pick->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
		return;
	}

	if(!grasps.empty())
	{
		ROS_INFO("PickGoalCB: Found %lu grasps for this object", grasps.size());
		for(unsigned int i=0; i<grasps.size(); i++)
		{
			ROS_DEBUG_STREAM("Grasp "<< i << ": " << grasps[i]);
		}
	}
	else
	{
		ROS_ERROR("No grasps found for object %s in database %s using gripper_type %s", goal->object_name.c_str(), goal->grasp_database.c_str(), goal->gripper_type.c_str());
		result.success.data=false;
		response="Pick failed: No grasps found!";
		as_pick->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
		return;
	}

	///Updating the object collision_object
	insertObject(goal->object_name, goal->object_class, goal->object_pose);

	if(!(goal->support_surface.empty()))
	{
		ROS_INFO("Setting SupportSurface to %s", goal->support_surface.c_str());
		group.setSupportSurfaceName(goal->support_surface);
	}

	///Call Pick
	group.setPlanningTime(300.0);	//default is 5.0 s
	moveit::planning_interface::MoveItErrorCode error_code = group.pick(goal->object_name, grasps);

	if(error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::string msg = "PICK SUCCEEDED: " + boost::lexical_cast<std::string>(error_code);
		ROS_INFO(msg);
		result.success.data=true;
		response=msg;
		as_pick->setSucceeded(result, response);
		last_grasp_valid = true;
		last_object_name = goal->object_name;
	}
	else
	{
		std::string msg = "PICK FAILED: " + boost::lexical_cast<std::string>(error_code);
		ROS_ERROR(msg);
		result.success.data=false;
		response=msg;
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

	std::vector<moveit_msgs::PlaceLocation> locations;
	trajectory_msgs::JointTrajectory pre_grasp_posture;		//use cyl_open by default
	pre_grasp_posture.header.stamp = ros::Time::now();
	pre_grasp_posture.joint_names.resize(7);
	pre_grasp_posture.joint_names[0] = "sdh_knuckle_joint";
	pre_grasp_posture.joint_names[1] = "sdh_finger_12_joint";
	pre_grasp_posture.joint_names[2] = "sdh_finger_13_joint";
	pre_grasp_posture.joint_names[3] = "sdh_finger_22_joint";
	pre_grasp_posture.joint_names[4] = "sdh_finger_23_joint";
	pre_grasp_posture.joint_names[5] = "sdh_thumb_2_joint";
	pre_grasp_posture.joint_names[6] = "sdh_thumb_3_joint";

	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.assign(7,0.0);
	point.velocities.assign(7,0.0);
	point.accelerations.assign(7,0.0);
	point.effort.assign(7,0.0);
	point.time_from_start = ros::Duration(3.0);

	point.positions[0] = 0.0;
	point.positions[1] = -0.9854;
	point.positions[2] = 0.9472;
	point.positions[3] = -0.9854;
	point.positions[4] = 0.9472;
	point.positions[5] = -0.9854;
	point.positions[6] = 0.9472;
	pre_grasp_posture.points.push_back(point);

	for(unsigned int i=0; i<goal->destinations.size(); i++)
	{
		moveit_msgs::PlaceLocation place_location;

		place_location.id = "Last_"+goal->object_name+"_grasp";
		place_location.post_place_posture = pre_grasp_posture;
		place_location.place_pose = goal->destinations[i];
		place_location.pre_place_approach.direction.header.frame_id = "/base_footprint";
		place_location.pre_place_approach.direction.vector.z = -1.0;
		place_location.pre_place_approach.min_distance = 0.1;
		place_location.pre_place_approach.desired_distance = 0.15;
		place_location.post_place_retreat.direction.header.frame_id = "/base_footprint";
		place_location.post_place_retreat.direction.vector.z = 1.0;
		place_location.post_place_retreat.min_distance = 0.1;
		place_location.post_place_retreat.desired_distance = 0.15;

		locations.push_back(place_location);
	}

    if(!(goal->support_surface.empty()))
	{
		ROS_INFO("Setting SupportSurface to %s", goal->support_surface.c_str());
		group.setSupportSurfaceName(goal->support_surface);
	}
	group.setPlanningTime(300.0);	//default is 5.0 s

	moveit::planning_interface::MoveItErrorCode error_code = group.place(goal->object_name, locations);

	if(error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::string msg = "PLACE SUCCEEDED: " + boost::lexical_cast<std::string>(error_code);
		ROS_INFO(msg.c_str());
		result.success.data=true;
		response=msg;
		as_place->setSucceeded(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
	}
	else
	{
		std::string msg = "PLACE FAILED: " + boost::lexical_cast<std::string>(error_code);
		ROS_ERROR(msg.c_str());
		result.success.data=false;
		response=msg;
		as_place->setAborted(result, response);
		last_grasp_valid = false;
		last_object_name.clear();
	}
}

void CobPickPlaceActionServer::insertObject(std::string object_name, unsigned int object_class, geometry_msgs::PoseStamped object_pose)
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

	std::string mesh_name = map_classid_to_classname[object_class];
	std::transform(mesh_name.begin(), mesh_name.end(), mesh_name.begin(), ::tolower);

	boost::scoped_ptr<shapes::Mesh> mesh;
	mesh.reset(shapes::createMeshFromResource("package://cob_grasp_generation/files/meshes/"+mesh_name+".stl"));
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(mesh.get(), shape_msg);
	co.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
	co.mesh_poses.push_back(object_pose.pose);
	pub_co.publish(co);


	tf::Transform transform;
	transform.setOrigin( tf::Vector3(object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z) );
	transform.setRotation( tf::Quaternion(object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w) );
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), object_pose.header.frame_id, object_name));


	ros::Duration(1.0).sleep();
}



void CobPickPlaceActionServer::fillAllGraspsKIT(unsigned int objectClassId, std::string gripper_type, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps)
{
	grasps.clear();
	Grasp *current_grasp = NULL;

	///Initialize GraspTable
	std::string path = ros::package::getPath("cob_grasp_generation")+std::string("/files/")+gripper_type+std::string("_grasptable_kit.txt");
	GraspTableIniFile = const_cast<char*>(path.c_str());
	m_GraspTable = new GraspTable();
	int error = m_GraspTable->Init(GraspTableIniFile);
	if(error<0)
	{
		ROS_ERROR("Failed to initialize GraspTables");
		return;
	}

	m_GraspTable->ResetReadPtr(objectClassId);
	unsigned int grasp_index = 0;

	current_grasp = m_GraspTable->GetNextGrasp(objectClassId);

	while(current_grasp)
	{
		convertGraspKIT(current_grasp, object_pose, grasps);

		current_grasp = m_GraspTable->GetNextGrasp(objectClassId);
		grasp_index++;
	}
}

void CobPickPlaceActionServer::fillSingleGraspKIT(unsigned int objectClassId, std::string gripper_type, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps)
{
	grasps.clear();
	Grasp *current_grasp = NULL;

	///Initialize GraspTable
	std::string path = ros::package::getPath("cob_grasp_generation")+std::string("/files/")+gripper_type+std::string("_grasptable_kit.txt");
	GraspTableIniFile = const_cast<char*>(path.c_str());
	m_GraspTable = new GraspTable();
	int error = m_GraspTable->Init(GraspTableIniFile);
	if(error<0)
	{
		ROS_ERROR("Failed to initialize GraspTables");
		return;
	}

	current_grasp = m_GraspTable->GetGrasp(objectClassId, grasp_id);

	if(current_grasp)
	{
		ROS_INFO("GraspIndex %d found",grasp_id);

		convertGraspKIT(current_grasp, object_pose, grasps);
	}
	else
		ROS_ERROR("Grasp %d NOT found", grasp_id);
}

void CobPickPlaceActionServer::convertGraspKIT(Grasp* current_grasp, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps)
{
	bool debug = true;
	moveit_msgs::Grasp g;

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

	g.pre_grasp_approach = calculateApproachDirection(msg_pose_grasp_FOOTPRINT_from_ARM7.pose, msg_pose_pre_FOOTPRINT_from_ARM7.pose);

	//~~~ RetreatDirection ~~~
	g.post_grasp_retreat.direction.header.frame_id = g.pre_grasp_approach.direction.header.frame_id;
	g.post_grasp_retreat.direction.vector.x = -g.pre_grasp_approach.direction.vector.x;
	g.post_grasp_retreat.direction.vector.y = -g.pre_grasp_approach.direction.vector.y;
	g.post_grasp_retreat.direction.vector.z = -g.pre_grasp_approach.direction.vector.z + 0.5; //also lift the object a little bit

	g.post_grasp_retreat.min_distance = g.pre_grasp_approach.min_distance;
	g.post_grasp_retreat.desired_distance = g.pre_grasp_approach.desired_distance;

	g.post_place_retreat = g.post_grasp_retreat;

	grasps.push_back(g);
}




void CobPickPlaceActionServer::fillGraspsOR(unsigned int objectClassId, std::string gripper_type, std::string gripper_side, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps)
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
	goal_query_grasps.gripper_type = gripper_type;
	goal_query_grasps.gripper_side = gripper_side;
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

		ROS_INFO("Found %lu grasps for this object", result_query_grasps.get()->grasp_list.size());
		for(unsigned int i=0; i<result_query_grasps.get()->grasp_list.size(); i++)
		{
			ROS_DEBUG_STREAM("Grasp "<< i << ": " << result_query_grasps.get()->grasp_list[i]);
		}

		for(unsigned int i=0; i<result_query_grasps.get()->grasp_list.size(); i++)
		{
			moveit_msgs::Grasp current_grasp;
			//~~~ HandGraspConfig ~~~
			current_grasp.grasp_posture = result_query_grasps.get()->grasp_list[i].grasp_posture;
			//for(unsigned int k=0; k<current_grasp.grasp_posture.points[0].positions.size(); k++)
			//{
				//if(current_grasp.grasp_posture.points[0].positions[k] < -1.5707) current_grasp.grasp_posture.points[0].positions[k] = -1.5707;
				//if(current_grasp.grasp_posture.points[0].positions[k] >  1.5707) current_grasp.grasp_posture.points[0].positions[k] =  1.5707;
			//}
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
			//current_grasp.pre_grasp_approach.direction.header.frame_id = "/base_footprint";
			//current_grasp.pre_grasp_approach.direction.vector.x = 0.0;
			//current_grasp.pre_grasp_approach.direction.vector.y = 0.0;
			//current_grasp.pre_grasp_approach.direction.vector.z = -1.0;
			//current_grasp.pre_grasp_approach.min_distance = 0.18;
			//current_grasp.pre_grasp_approach.desired_distance = 0.28;


			ROS_INFO_STREAM("EndeffectorLink: " << group.getEndEffectorLink());
			current_grasp.pre_grasp_approach.direction.header.frame_id = group.getEndEffectorLink();
			current_grasp.pre_grasp_approach.direction.vector.x = 0.0;
			current_grasp.pre_grasp_approach.direction.vector.y = 0.0;
			current_grasp.pre_grasp_approach.direction.vector.z = 1.0;
			current_grasp.pre_grasp_approach.min_distance = 0.18;
			current_grasp.pre_grasp_approach.desired_distance = 0.28;

			//current_grasp.pre_grasp_approach.direction.header.frame_id = "/base_footprint";
			//current_grasp.pre_grasp_approach.direction.vector.x = -msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.x+object_pose.pose.position.x;
			//current_grasp.pre_grasp_approach.direction.vector.y = -msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.y+object_pose.pose.position.y;
			//current_grasp.pre_grasp_approach.direction.vector.z = -msg_pose_grasp_FOOTPRINT_from_ARM7.pose.position.z+object_pose.pose.position.z;
			//current_grasp.pre_grasp_approach.min_distance = 0.18;
			//current_grasp.pre_grasp_approach.desired_distance = 0.28;

			//~~~ RetreatDirection ~~~
			current_grasp.post_grasp_retreat.direction.header.frame_id = "/base_footprint";
			current_grasp.post_grasp_retreat.direction.vector.x = 0.0;
			current_grasp.post_grasp_retreat.direction.vector.y = 0.0;
			current_grasp.post_grasp_retreat.direction.vector.z = 1.0;
			current_grasp.post_grasp_retreat.min_distance = 0.05;
			current_grasp.post_grasp_retreat.desired_distance = 0.1;

			// current_grasp.post_grasp_retreat.direction.header.frame_id = "/arm_7_link";
			// current_grasp.post_grasp_retreat.direction.vector.x = 0.0;
			// current_grasp.post_grasp_retreat.direction.vector.y = 0.0;
			// current_grasp.post_grasp_retreat.direction.vector.z = -1.0;
			// current_grasp.post_grasp_retreat.min_distance = 0.1;
			// current_grasp.post_grasp_retreat.desired_distance = 0.15;

			current_grasp.post_place_retreat = current_grasp.post_grasp_retreat;

			grasps.push_back(current_grasp);
		}

	}
	else
		ROS_ERROR("Grasps not queried within timeout");
}






trajectory_msgs::JointTrajectory CobPickPlaceActionServer::MapHandConfiguration(sensor_msgs::JointState table_config)
{
	trajectory_msgs::JointTrajectory grasp_configuration;

	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.assign(7,0.0);
	point.velocities.assign(7,0.0);
	point.accelerations.assign(7,0.0);
	point.effort.assign(7,0.0);
	point.time_from_start = ros::Duration(3.0);

	point.positions[0] = table_config.position[0];
	point.positions[1] = table_config.position[2];
	point.positions[2] = table_config.position[3];
	point.positions[3] = table_config.position[5];
	point.positions[4] = table_config.position[6];
	point.positions[5] = table_config.position[11];
	point.positions[6] = table_config.position[12];
	grasp_configuration.points.push_back(point);

	grasp_configuration.joint_names.resize(7);
	grasp_configuration.joint_names[0]=("sdh_knuckle_joint");
	grasp_configuration.joint_names[1]=("sdh_thumb_2_joint");
	grasp_configuration.joint_names[2]=("sdh_thumb_3_joint");
	grasp_configuration.joint_names[3]=("sdh_finger_12_joint");
	grasp_configuration.joint_names[4]=("sdh_finger_13_joint");
	grasp_configuration.joint_names[5]=("sdh_finger_22_joint");
	grasp_configuration.joint_names[6]=("sdh_finger_23_joint");

	//cut joint_values according to joint_limits
	for(unsigned int i=0; i<grasp_configuration.points[0].positions.size();i++)
	{
		if(grasp_configuration.points[0].positions[i]>1.57079){grasp_configuration.points[0].positions[i]=1.57079;}
		if(grasp_configuration.points[0].positions[i]<-1.57079){grasp_configuration.points[0].positions[i]=-1.57079;}
	}
	return grasp_configuration;
}

tf::Transform CobPickPlaceActionServer::transformPose(tf::Transform transform_O_from_SDH, tf::Transform transform_HEADER_from_O, std::string object_frame_id)
{
	bool debug = false;

	// SDH_from_ARM7
	tf::StampedTransform transform_SDH_from_ARM7;

	bool transform_available = false;
	while(!transform_available)
	{
		try{
			/// ToDo: get palm-link name from robot!
			//tf_listener_.lookupTransform("/sdh_palm_link", group.getEndEffectorLink(), ros::Time(0), transform_SDH_from_ARM7);
			tf_listener_.lookupTransform("/gripper_left_palm_link", group.getEndEffectorLink(), ros::Time(0), transform_SDH_from_ARM7);
			transform_available = true;
		}
		catch (tf::TransformException ex){
			ROS_WARN("Waiting for transform...(%s)",ex.what());
			ros::Duration(0.1).sleep();
		}
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

moveit_msgs::GripperTranslation CobPickPlaceActionServer::calculateApproachDirection(geometry_msgs::Pose msg_pose_grasp_FOOTPRINT_from_ARM7, geometry_msgs::Pose msg_pose_pre_FOOTPRINT_from_ARM7)
{
	double finger_length = 0.18; //this is the lenght of the sdh ('home' configuration)
	moveit_msgs::GripperTranslation approach;
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
	if( argc != 2 )
	{
		ROS_ERROR("No manipulator specified!");
		return -1;
	}
	else
	{
		ROS_INFO("Starting PickPlace ActionServer for group %s", argv[1]);
	}

	CobPickPlaceActionServer *cob_pick_place_action_server = new CobPickPlaceActionServer(std::string(argv[1]));

	cob_pick_place_action_server->initialize();
	cob_pick_place_action_server->run();

	return 0;
}
