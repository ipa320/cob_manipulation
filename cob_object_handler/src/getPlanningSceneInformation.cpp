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
 *   Adds cylinder as a known obstacle to the environment server.
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

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "addCylinder");
	ros::NodeHandle nh;

	ros::Publisher object_in_map_pub_;
	object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

	ros::Duration(2.0).sleep();
	
	ros::service::waitForService(GET_PLANNING_SCENE_NAME);
	ros::ServiceClient get_planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
	
	arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
	arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;
	
	if(!get_planning_scene_client.call(get_planning_scene_req, get_planning_scene_res)) 
	{
		ROS_WARN("Can't get planning scene");
		return -1;
	}
	ROS_INFO("Got planning_scene!");
	
	//ROS_INFO("AllowedCollisionMatrix.size: %d", get_planning_scene_res.planning_scene.allowed_collision_matrix.link_names.size());
	ROS_INFO("CollisionObjects: %d", get_planning_scene_res.planning_scene.collision_objects.size());
	ROS_INFO("AttachedCollisionObjects: %d", get_planning_scene_res.planning_scene.attached_collision_objects.size());
	
	ros::Duration(2.0).sleep();
	ros::shutdown();
}

