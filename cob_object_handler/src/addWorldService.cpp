/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
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
 *   ROS package name: cob_object_handler
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: April 2012
 *
 * \brief
 *   Adds the current environment as a known obstacle to the environment server.
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




#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <tinyxml.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>
#include <urdf/model.h>
#include <planning_models/kinematic_model.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <planning_environment/util/construct_object.h>
#include <gazebo/GetModelState.h>
//#include <urdf/link.h>

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/collision_models.h>
#include <cob_object_handler/HandleObject.h>
#include "geometric_shapes/shape_operations.h"

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

class AddWorldService
{
private:
	ros::NodeHandle nh;
	
	std::string frame_id;
	
	std::string parameter_name;
	std::string model_name;
	
	ros::Publisher object_in_map_pub_;
	
	ros::ServiceClient get_planning_scene_client;
	ros::ServiceClient set_planning_scene_diff_client;
	ros::ServiceClient get_model_state_client;
	
	arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
	arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;	
	
	ros::ServiceServer add_world_service;
	
	bool use_gazebo;
		
	
public:	
	AddWorldService()
	{
		//const std::string frame_id = "/base_footprint";

		parameter_name = "world_description";
		model_name = "urdf_world_model";
		
		object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 20);
	  
		//this sleep is important!
		ros::Duration(2.0).sleep();
		
		ROS_WARN("waiting for services...");
		ros::service::waitForService(GET_PLANNING_SCENE_NAME);
		get_planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
		set_planning_scene_diff_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

		use_gazebo = true;
		if(!ros::service::waitForService("/gazebo/get_model_state", 500))
		{
			ROS_WARN("/gazebo/get_model_state is not available...assuming to run on real robot");
			use_gazebo = false;
			frame_id = "/map";
			//frame_id = "/odom_combined";		
		}
		else
		{
			//access to tranformation /world to /root_link (table_top)
			get_model_state_client = nh.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
			frame_id = "/odom_combined";
		}
		ROS_INFO("...done!");

		add_world_service = nh.advertiseService("/addWorld", &AddWorldService::addWorld, this);
	}

	void run()
	{
		ROS_INFO("Ready to addWorld.");
		ros::spin();
	}

private:
	shapes::Shape* constructShape(const urdf::Geometry *geom)
	{
		ROS_ASSERT(geom);

		shapes::Shape *result = NULL;
		if(geom->type == urdf::Geometry::BOX)
		{
			ROS_DEBUG("BOX");
			urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
			result = new shapes::Box(dim.x, dim.y, dim.z);
		}
		else if(geom->type == urdf::Geometry::SPHERE)
		{
			ROS_DEBUG("SPHERE");
			result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
		}
		else if(geom->type == urdf::Geometry::CYLINDER)
		{
			ROS_DEBUG("CYLINDER");
			result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius, dynamic_cast<const urdf::Cylinder*>(geom)->length);
		}
		else if(geom->type == urdf::Geometry::MESH)
		{
			const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
			if (!mesh->filename.empty())
			{
				const btVector3* scale = new btVector3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
				result = shapes::createMeshFromFilename(mesh->filename.c_str(), scale);
		
				if (result == NULL)
					ROS_ERROR("Failed to load mesh '%s'", mesh->filename.c_str());
				else
					ROS_DEBUG("Loaded mesh '%s' consisting of %d triangles", mesh->filename.c_str(), static_cast<shapes::Mesh*>(result)->triangleCount);			
			}
			else
				ROS_WARN("Empty mesh filename");
		}
		else
		{
			ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
		}

		return result;
	}



	bool addWorld(cob_object_handler::HandleObject::Request  &req,
				  cob_object_handler::HandleObject::Response &res )
	{
		while(!nh.hasParam(parameter_name))
		{
			ROS_WARN("waiting for parameter \"world_description\"... ");
			ros::Duration(0.5).sleep();
		}
		  
		urdf::Model model;
		if (!model.initParam(parameter_name))
		{
			ROS_ERROR("Failed to parse %s from parameter server", parameter_name.c_str());
			return -1;
		}

		ROS_INFO("Successfully parsed urdf file");

		std::vector< boost::shared_ptr< urdf::Link > > URDF_links;
		model.getLinks(URDF_links);
		std::map< std::string, boost::shared_ptr< urdf::Joint > > URDF_joints = model.joints_;
		std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator joints_it;


		//tranfo between links is in joints->origin!
		//access to Joint-Information
		for(joints_it=URDF_joints.begin() ; joints_it != URDF_joints.end(); joints_it++)
		{
			ROS_DEBUG("Joint name: %s", (*joints_it).first.c_str());
			ROS_DEBUG("\t origin: %f,%f,%f", (*joints_it).second->parent_to_joint_origin_transform.position.x, (*joints_it).second->parent_to_joint_origin_transform.position.y, (*joints_it).second->parent_to_joint_origin_transform.position.z);
		}

		  
		joints_it=URDF_joints.begin();
		for(unsigned int i=0; i<URDF_links.size(); i++)
		{
			urdf::Link current_link = *URDF_links[i];
			ROS_DEBUG("Current Link: %s", current_link.name.c_str());
			
			arm_navigation_msgs::CollisionObject collision_object;
			collision_object.id = current_link.name.c_str();
			collision_object.header.frame_id = frame_id;
			collision_object.header.stamp = ros::Time::now();
				
			if(req.operation=="add") 
			{
				collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

				tf::Transform model_origin;
				model_origin.setIdentity();
				if(use_gazebo)
				{
					//access to tranformation /world to /root_link (table_top)
					gazebo::GetModelState srv;
					srv.request.model_name = model_name;
					if (get_model_state_client.call(srv))	{
						ROS_DEBUG("URDFPose (x,y,z): (%f,%f,%f)", srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z);
						model_origin = tf::Transform(tf::Quaternion(srv.response.pose.orientation.x, srv.response.pose.orientation.y, srv.response.pose.orientation.z, srv.response.pose.orientation.w), tf::Vector3(srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z));
					}
					else {
						ROS_ERROR("Failed to call service get_model_state");
						ros::shutdown();
					}
				}				
				else
				{
					/*
					tf::TransformListener listener;
					tf::StampedTransform transform;
					try
					{
						std::string error_string;
						ROS_INFO("waiting...");
						listener.waitForTransform("odom_combined", "map", ros::Time(0), ros::Duration(10000));
						ROS_INFO("done waiting for transform");
						//int success = listener.getLatestCommonTime("/odom_combined", "/map", time, &error_string);
						listener.lookupTransform("odom_combined", "map", ros::Time(0), transform);
					}
					catch (tf::TransformException ex)
					{
						ROS_ERROR("%s",ex.what());
					}
					model_origin.setBasis(transform.getBasis());
					model_origin.setOrigin(transform.getOrigin());
					double roll, pitch, yaw;
					model_origin.getBasis().getRPY(roll, pitch, yaw);
					ROS_INFO("Rotation: %f, %f, %f", roll, pitch, yaw);
					*/
				}


				tf::Pose pose;
				tf::Pose pose2;
				tf::Transform link_origin;
				tf::Transform joint_origin;
				
				link_origin = tf::Transform(tf::Quaternion(current_link.collision->origin.rotation.x, 
														   current_link.collision->origin.rotation.y,
														   current_link.collision->origin.rotation.z,
														   current_link.collision->origin.rotation.w),
											tf::Vector3(current_link.collision->origin.position.x,
														current_link.collision->origin.position.y,
														current_link.collision->origin.position.z));
				
				if(current_link.parent_joint == NULL)
				{
					ROS_DEBUG("Link does not have a parent joint...");
					joint_origin.setIdentity();		
					continue;		
				}
				else
				{			
				
					boost::shared_ptr< urdf::Joint > current_parent_joint = current_link.parent_joint;
					ROS_DEBUG("Current Parent Joint: %s", current_parent_joint->name.c_str());
					joint_origin = tf::Transform(tf::Quaternion(current_parent_joint->parent_to_joint_origin_transform.rotation.x, 
														current_parent_joint->parent_to_joint_origin_transform.rotation.y, 
														current_parent_joint->parent_to_joint_origin_transform.rotation.z, 
														current_parent_joint->parent_to_joint_origin_transform.rotation.w), 
										   tf::Vector3(current_parent_joint->parent_to_joint_origin_transform.position.x, 
													   current_parent_joint->parent_to_joint_origin_transform.position.y, 
													   current_parent_joint->parent_to_joint_origin_transform.position.z));
				}
				
				tf::Transform temp;
				temp.mult(joint_origin, link_origin);			
				pose2.mult(model_origin, temp);

				tf::Stamped<tf::Pose> stamped_pose_in;
				stamped_pose_in.stamp_ = ros::Time::now();
				stamped_pose_in.frame_id_ = frame_id;
				stamped_pose_in.setData(pose2);


				//fill CollisionObject for each link
				shapes::Shape *current_shape;
				current_shape = constructShape(current_link.collision->geometry.get());
				ROS_DEBUG("shape.type: %d", current_shape->type);
				ROS_DEBUG("Position (x,y,z): (%f,%f,%f)", current_link.collision->origin.position.x, current_link.collision->origin.position.y, current_link.collision->origin.position.z);

				
				arm_navigation_msgs::Shape msg_shape;
				planning_environment::constructObjectMsg(current_shape, msg_shape);

				geometry_msgs::PoseStamped msg_pose_stamped;
				tf::poseStampedTFToMsg (stamped_pose_in, msg_pose_stamped);

				collision_object.shapes.push_back(msg_shape);
				collision_object.poses.push_back(msg_pose_stamped.pose);

			}
			else if (req.operation=="remove")
			{
				ROS_INFO("Removing the world");
				collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
			}
			else
			{
				ROS_ERROR("Unknown operation! Please use 'add' or 'remove'");
				return false;
			}
			
			object_in_map_pub_.publish(collision_object);
			ros::Duration(0.1).sleep();
		}
			
		arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
		arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
		
		if(!set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
		{	
			ROS_ERROR("Can't get planning scene");
		}
		
		ROS_INFO("Should have published");
			

	  return true;
	}
	
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "addWorldServiceServer");
	
	AddWorldService* add_world_service = new AddWorldService();

	add_world_service->run();

	return 0;
}


