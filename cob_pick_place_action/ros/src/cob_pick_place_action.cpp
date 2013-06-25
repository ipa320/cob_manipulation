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
#include <cob_pick_place_action/cob_pick_place_action.h>
#include <geometric_shapes/shape_operations.h>
#include <math.h>

//~ COBPickAction::COBPickAction(){}

void COBPickAction::initialize()
{
	pub_co = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	attached_object_publisher = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
	static const std::string COB_PICKUP_ACTION = "COBPickUp"; // name of 'pickup' action
	cob_pick_action_server.reset(new actionlib::SimpleActionServer<cob_pick_place_action::COBPickUpAction>(nh_, COB_PICKUP_ACTION, boost::bind(&COBPickAction::goalCB, this, _1), false));                                                                                          
	cob_pick_action_server->start();   	
	
	//~ Grasp Table Initializations################################
	//~ GraspTableIniFile="/home/fxm-rc/groovy/care-o-bot/cob_manipulation/cob_pick_place_action/files/GraspTable.txt";
	std::string path = ros::package::getPath("cob_pick_place_action")+std::string("/files/GraspTable.txt");
	GraspTableIniFile=const_cast<char*>(path.c_str()); 
	m_GraspTable = new GraspTable();
	int error = m_GraspTable->Init(GraspTableIniFile);
}
	
void COBPickAction::goalCB(const cob_pick_place_action::COBPickUpGoalConstPtr &goal)
{
	setUpEnvironment();
   
    std::vector<manipulation_msgs::Grasp> grasps;
    unsigned int objectClassId=11;
	fillGrasps(objectClassId, grasps);  
	ROS_WARN_STREAM("cob_tutotrial::Grasp_0 info : " << grasps[0] );
	//~ ROS_WARN_STREAM("cob_tutotrial::Grasp_1 info : " << grasps[1] );
	ROS_INFO("size of grasps[]:= %d", grasps.size());	
	group.setSupportSurfaceName("table");

	ROS_INFO("PICK CALLED");
    bool success=group.pick("part", grasps);
    cob_pick_place_action::COBPickUpResult action_res;
	setCOBPickupResponse(action_res, success);
}
	

void COBPickAction::setCOBPickupResponse(cob_pick_place_action::COBPickUpResult &action_res, bool success)
{
	std::string response;
	if(success)
	{
		action_res.success.data=true;
		response="Hurray!!! Pickup COMPLETE";
		cob_pick_action_server->setSucceeded(action_res, response);
	}
	else
	{
		action_res.success.data=false;
		response="Alas!!! Pickup FAILED";
		cob_pick_action_server->setAborted(action_res, response);
	}
}



void COBPickAction::setUpEnvironment()
{
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "part";
  detach_object.link_name = "arm_7_link";
  detach_object.object.operation = detach_object.object.REMOVE;
  attached_object_publisher.publish(detach_object);  


  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";

  //~ making mesh##################################################
  //~ const tf::Vector3 scale(0.001,0.001,0.001);
  
  
  //~ Adding to the collision object#####################################
  //~ co.id = "part1";
  //~ co.operation = co.REMOVE;
  //~ pub_co.publish(co);
  //~ co.operation = co.ADD;
  //~ 
  //~ boost::scoped_ptr<shapes::Mesh> mesh ;
  //~ mesh.reset(shapes::createMeshFromResource("package://cob_pick_place_action/Sauerkraut_5k.stl"));
  //~ mesh->scale(0.001);
  //~ if (mesh) 
  //~ {
    //~ shapes::ShapeMsg shape_msg;
    //~ shapes::constructMsgFromShape(mesh.get(), shape_msg);    
    //~ co.meshes.resize(1);
    //~ co.meshes[0]= boost::get<shape_msgs::Mesh>(shape_msg);
  //~ }
  //~ 
  //~ co.mesh_poses.resize(1);
  //~ co.mesh_poses[0].position.x = 0;
  //~ co.mesh_poses[0].position.y = 0;  
  //~ co.mesh_poses[0].position.z = 0;
  //~ co.mesh_poses[0].orientation.w = 1.0;
  //~ //co.mesh_poses[0].position.x = -0.749;
  //~ //co.mesh_poses[0].position.y = -0.903;  
  //~ //co.mesh_poses[0].position.z = 0.893;
  //~ 
  //~ pub_co.publish(co);
  //~ Published###########################################################
  
  // remove pole
  co.id = "pole";
  co.operation = co.REMOVE;
  pub_co.publish(co);
  
  // add pole
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


  ROS_INFO("inside setUpEnvironment");
  // remove table
  co.id = "table";
  co.operation = co.REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = co.ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.2;  
  co.primitive_poses[0].position.z = 0.175;
  pub_co.publish(co);
  


  co.id = "part";
  co.operation = co.REMOVE;
  pub_co.publish(co);

  co.operation = co.ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  co.primitive_poses[0].position.x = -0.5;
  co.primitive_poses[0].position.y = -0.5;  
  co.primitive_poses[0].position.z = 0.6;
  pub_co.publish(co);
  ros::spinOnce();
  ros::Duration(0.1).sleep();
}

void COBPickAction::fillGrasps(unsigned int objectClassId, std::vector<manipulation_msgs::Grasp> &grasps)
{  
  grasps.clear(); 
  manipulation_msgs::Grasp g;
  	
  ROS_INFO("fillGrasps CALLED 1");
  Grasp * current_grasp = NULL;
  int grasp_index = 0;
  bool new_situation= true;
  
  current_grasp = m_GraspTable->GetNextGrasp(objectClassId, new_situation);
  new_situation= false;
  
  //~ while(current_grasp)
  while(grasp_index<1)
  {	
	ROS_INFO("index is %d",grasp_index);	  
	
	//~ ROS_INFO("current_hand_pre_config");	
    std::vector<double> current_hand_pre_config = current_grasp->GetHandPreGraspConfig();
    sensor_msgs::JointState pre_grasp_posture;    
    pre_grasp_posture.position.clear();
	for (int ii=0; ii< current_hand_pre_config.size();ii++)
	{
		pre_grasp_posture.position.push_back(current_hand_pre_config[ii]);
	}	
    g.pre_grasp_posture= MapHandConfiguration(pre_grasp_posture);	
    
	//~ ROS_INFO("current_hand_config");
	std::vector<double> current_hand_config = current_grasp->GetHandGraspConfig();
	sensor_msgs::JointState grasp_posture;
	grasp_posture.position.clear();
	for (int ii=0; ii< current_hand_config.size();ii++)
	{
		grasp_posture.position.push_back(current_hand_config[ii]);
	}
	g.grasp_posture= MapHandConfiguration(grasp_posture);	
	
	    //~ get this pose from object recognition. For now put the hard coded values##################
	geometry_msgs::Pose pose_of_object_recognition;
	pose_of_object_recognition.position.x = -0.5;
	pose_of_object_recognition.position.y = -0.5;  
	pose_of_object_recognition.position.z = 0.6;
	pose_of_object_recognition.orientation=tf::createQuaternionMsgFromYaw(0);
	
	//~Might have to transform here if frame is not proper for object recognition and  
	geometry_msgs::Pose object_pose_base_footprint;
	object_pose_base_footprint=pose_of_object_recognition;

   //current_hand_pose############################################~ 
	std::vector<double> current_hand_pose = current_grasp->GetTCPGraspPose();
	geometry_msgs::Pose pose_grasp_wrt_object;	
	pose_grasp_wrt_object.position.x=current_hand_pose[0];
	pose_grasp_wrt_object.position.y=current_hand_pose[1];
	pose_grasp_wrt_object.position.z=current_hand_pose[2];
	pose_grasp_wrt_object.orientation=tf::createQuaternionMsgFromRollPitchYaw(current_hand_pose[3], current_hand_pose[4], current_hand_pose[5] );
	
	//~ Get grasp_pose in Base_footprint
	g.grasp_pose.pose=GraspPoseWRTBaseFootprint(pose_grasp_wrt_object, object_pose_base_footprint);		
	g.grasp_pose.header.frame_id = "base_footprint";
		
	//~ ROS_INFO("current_hand_pre_grasp_pose");
    std::vector<double> current_hand_pre_pose = current_grasp->GetTCPPreGraspPose();
    geometry_msgs::Pose pre_pose_grasp_wrt_object;	
    pre_pose_grasp_wrt_object.position.x=current_hand_pre_pose[0];
	pre_pose_grasp_wrt_object.position.y=current_hand_pre_pose[1];
	pre_pose_grasp_wrt_object.position.z=current_hand_pre_pose[2];
	pre_pose_grasp_wrt_object.orientation=tf::createQuaternionMsgFromRollPitchYaw(current_hand_pre_pose[3], current_hand_pre_pose[4], current_hand_pre_pose[5] );
	
	geometry_msgs::Pose pre_pose_grasp_wrt_baseFootprint=GraspPoseWRTBaseFootprint(pre_pose_grasp_wrt_object, object_pose_base_footprint);
  //~ ROS_INFO("Getting approach direction and pose");
	g.approach=getGraspApproachData(g.grasp_pose.pose, pre_pose_grasp_wrt_baseFootprint);
	
	
	//~ g.approach.direction.vector.z = 1.0;
	//~ g.approach.direction.header.frame_id = "sdh_palm_link";
	//~ g.approach.min_distance = 0.2;
	//~ g.approach.desired_distance = 0.4;

	g.retreat.direction.header.frame_id = "base_footprint";
	g.retreat.direction.vector.z = 1.0;
	g.retreat.min_distance = 0.1;
	g.retreat.desired_distance = 0.25;


	grasps.push_back(g);
	current_grasp = m_GraspTable->GetNextGrasp(objectClassId, new_situation);
		
	if (current_grasp == NULL)
	{
		ROS_INFO("GRASP TABLE EMPTY");
	}
	
	grasp_index=grasp_index+1;
  }
  grasps.resize(grasp_index);
}

manipulation_msgs::GripperTranslation COBPickAction::getGraspApproachData(geometry_msgs::Pose current_hand_pose, geometry_msgs::Pose current_hand_pre_pose)
{
	manipulation_msgs::GripperTranslation approach;
	approach.direction.header.frame_id = "base_footprint";
	//~ dis=sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2)
	//~ direction.x= (x1-x0)/dis and likewise
	approach.desired_distance=sqrt(pow((current_hand_pose.position.x-current_hand_pre_pose.position.x),2)+pow((current_hand_pose.position.y-current_hand_pre_pose.position.y),2)+pow((current_hand_pose.position.z-current_hand_pre_pose.position.z),2));
	approach.min_distance = 0.1;
	
	approach.direction.vector.x = (current_hand_pose.position.x-current_hand_pre_pose.position.x)/approach.desired_distance;
	approach.direction.vector.y = (current_hand_pose.position.y-current_hand_pre_pose.position.y)/approach.desired_distance;
	approach.direction.vector.z = (current_hand_pose.position.z-current_hand_pre_pose.position.z)/approach.desired_distance;
	
	if((approach.min_distance+0.1)>approach.desired_distance)
	approach.desired_distance=approach.min_distance+0.1;
	
	ROS_WARN_STREAM("approach info : " << approach );
	return approach;
}

void COBPickAction::Run()
{	
	ROS_INFO("cob_pick_action...spinning");
    ros::spin();
}	


sensor_msgs::JointState COBPickAction::MapHandConfiguration(sensor_msgs::JointState table_config)
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
//#################################################################################################################################################################################
//                   Joint position brought to 1.57079 from 1.5708 to get proper joint limit check
//#################################################################################################################################################################################
	for(int ik=0; ik<grasp_configuration.position.size();ik++)
	{
		if(grasp_configuration.position[ik]>1.57079){grasp_configuration.position[ik]=1.57079;}
		if(grasp_configuration.position[ik]<-1.57079){grasp_configuration.position[ik]=-1.57079;}
	}
	return grasp_configuration;
}



geometry_msgs::Pose COBPickAction::GraspPoseWRTBaseFootprint(geometry_msgs::Pose grasp_pose, geometry_msgs::Pose pose_of_object_recognition)
{
	tf::Quaternion tf_Quaternion;
	tf::quaternionMsgToTF(pose_of_object_recognition.orientation,tf_Quaternion);
	tf::Vector3 tf_vector_3= tf::Vector3(pose_of_object_recognition.position.x, pose_of_object_recognition.position.y, pose_of_object_recognition.position.z);
	tf::Transform transformation_object_to_BaseFootprint = tf::Transform(tf_Quaternion, tf_vector_3);
	
	tf::quaternionMsgToTF(grasp_pose.orientation,tf_Quaternion);
	tf_vector_3= tf::Vector3(grasp_pose.position.x/1000, grasp_pose.position.y/1000, grasp_pose.position.z/1000);
	tf::Transform transformation_grasp_to_object = tf::Transform(tf_Quaternion, tf_vector_3);
	
	
    tf::Transform transformation_grasp_to_odom_combined= transformation_object_to_BaseFootprint.operator*(transformation_grasp_to_object );
    
    geometry_msgs::Transform msg;
    tf::transformTFToMsg (transformation_grasp_to_odom_combined,msg );
	
	geometry_msgs::Pose transformed_FromObject_ToOdom_combined_grasp_pose;
	transformed_FromObject_ToOdom_combined_grasp_pose.position.x=msg.translation.x;
	transformed_FromObject_ToOdom_combined_grasp_pose.position.y=msg.translation.y;
	transformed_FromObject_ToOdom_combined_grasp_pose.position.z=msg.translation.z;

	transformed_FromObject_ToOdom_combined_grasp_pose.orientation=msg.rotation;
	return transformed_FromObject_ToOdom_combined_grasp_pose;
	
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_pick_action");
	COBPickAction*  cob_pick_action= new COBPickAction();
	cob_pick_action->initialize();
	cob_pick_action->Run();
	return 0;
}
