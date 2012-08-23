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

#include <arm_navigation_msgs/CollisionObject.h>

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";


int main(int argc, char **argv)
{
	ros::init(argc, argv, "collisionObjectsModifier");

	ros::NodeHandle nh;
	unsigned int index, first, last;
	
	if (argc < 2)
	{
		ROS_ERROR("Please give index of to be removed");
		return -1;
	}
	else if (argc >3)
	{
		ROS_ERROR("To many parameters");
		return -1;
	}
		
	ros::service::waitForService(GET_PLANNING_SCENE_NAME);
	ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
	
	ros::ServiceClient get_planning_scene_client;
	arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
	arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;
	get_planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
	
	ros::ServiceClient set_planning_scene_diff_client;
	arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
	arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
	set_planning_scene_diff_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
	
	ros::Publisher object_in_map_pub_;
	object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 20);
	
	
	ros::Duration(2.0).sleep();
	
	
	if(!get_planning_scene_client.call(get_planning_scene_req, get_planning_scene_res)) 
	{
		ROS_WARN("Can't get planning scene");
		return -1;
	}
	ROS_INFO("Got planning_scene!");
	
	
	for(unsigned int i=0; i<get_planning_scene_res.planning_scene.collision_objects.size(); i++)
	{
		if(get_planning_scene_res.planning_scene.collision_objects[i].id == "urdf_world_model_object")
		{
			ROS_INFO("Found the world within the collision_objects");
			arm_navigation_msgs::CollisionObject world_object = get_planning_scene_res.planning_scene.collision_objects[i];
			
			if (argc == 2)
			{
				index = atoi(argv[1]);
				if(world_object.shapes.size() < index)
				{
					ROS_ERROR("Index out of bounds");
					return -1;
				}

				world_object.shapes.erase(world_object.shapes.begin() + index);
				world_object.poses.erase(world_object.poses.begin() + index);					
			}
			else if (argc == 3)
			{
				first = atoi(argv[1]);
				last = atoi(argv[2]);
				if((first > last) or (world_object.shapes.size() < first) or (world_object.shapes.size() < last))
				{
					ROS_ERROR("Invalid parameter");
					return -1;
				}
				
				world_object.shapes.erase(world_object.shapes.begin()+first, world_object.shapes.begin()+last);
				world_object.poses.erase(world_object.poses.begin()+first, world_object.poses.begin()+last);								
			}
			

			object_in_map_pub_.publish(world_object);
			
			ros::Duration(2.0).sleep();
			
			if(!set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
			{	
				ROS_ERROR("Can't set planning scene");
			}		
			ROS_INFO("Should have published");
			
			return 0;
		}
	}
	
	ROS_ERROR("Can't find the world in the collision_objects");
	return -1;
}


