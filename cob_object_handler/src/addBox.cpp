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
 *   Adds box as a known obstacle to the environment server.
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

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "addBox");
	ros::NodeHandle nh;

	ros::Publisher object_in_map_pub_;
	object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

	ros::Duration(2.0).sleep();
	
	ros::service::waitForService(GET_PLANNING_SCENE_NAME);
	ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
	ros::ServiceClient get_planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
	ros::ServiceClient set_planning_scene_diff_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
	
	arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
	arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;
	
	if(argc > 1) 
	{
		std::stringstream s(argv[1]);
		bool add;
		s >> add;

		if(add) 
		{
			ROS_INFO("Adding the obstacle");
			//add the box into the collision space
			arm_navigation_msgs::CollisionObject box_object;
			box_object.id = "box";
			//cylinder_object.padding = 10.0;
			box_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
			//cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
			box_object.header.frame_id = "/map";
			box_object.header.stamp = ros::Time::now();
			arm_navigation_msgs::Shape object;
			object.type = arm_navigation_msgs::Shape::BOX;
			object.dimensions.resize(3);
			object.dimensions[0] = 0.05;
			object.dimensions[1] = 1.6;
			object.dimensions[2] = 0.9;
			geometry_msgs::Pose pose;
			pose.position.x = 4.229;
			pose.position.y = -.0;
			pose.position.z = .45;
			pose.orientation.x = 0;
			pose.orientation.y = 0;
			pose.orientation.z = 0;
			pose.orientation.w = 1;
			box_object.shapes.push_back(object);
			box_object.poses.push_back(pose);
			
			object_in_map_pub_.publish(box_object);
			
			arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
			arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
			
			if(!set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
			{	
				ROS_ERROR("Can't get planning scene");
			}
			
			ROS_INFO("Got planning_scene!");
			
			ROS_INFO("Should have published");
		}
		else
		{
			ROS_INFO("Removing the pole");
			if(!get_planning_scene_client.call(get_planning_scene_req, get_planning_scene_res)) 
			{
				ROS_WARN("Can't get planning scene");
				return -1;
			}
			ROS_INFO("Got planning_scene!");
			
			for(unsigned int i=0; i<get_planning_scene_res.planning_scene.collision_objects.size(); i++)
			{
				if(get_planning_scene_res.planning_scene.collision_objects[i].id == "box")
				{
					ROS_INFO("Found the pole within the collision_objects");
					arm_navigation_msgs::CollisionObject box_object = 	get_planning_scene_res.planning_scene.collision_objects[i];
					box_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
					
					object_in_map_pub_.publish(box_object);
					ROS_INFO("Should have published");
					break;
				}
			}
		}
	}
	else
		ROS_WARN("Please call with argument: 1->addPole; 0->removePole");
		
	
	
	ros::Duration(2.0).sleep();
	ros::shutdown();
}

