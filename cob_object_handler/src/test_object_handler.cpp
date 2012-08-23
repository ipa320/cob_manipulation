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
 *   This package provides services for handling a graspable object. 
 *   It reads data from the parameter_server in order to add it to or remove it from the environment_server as a known obstacle.
 * 	 Also it can attach such object to the robot in order to consider it as a part of the robot during the planning process.
 *
 ****************************************************************/




#include <ros/ros.h>
#include <cob_object_handler/HandleObject.h>

#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_object_handler");
	if (argc != 3)
	{
		ROS_INFO("usage: test_object_handler <operation> <object_name>");
		ROS_INFO("----------------------------------------------------");
		ROS_INFO("available operations are:");
		ROS_INFO("- add");
		ROS_INFO("- remove");
		ROS_INFO("- attach");
		ROS_INFO("- detach");
		ROS_INFO("----------------------------------------------------");
		ROS_INFO("Note: for the given <object_name> there should be a /<object_name>_description on the parameter_serveer and the model should be called <object_name>_model");
		return 1;
	}
	std::string operation = argv[1];

	ros::NodeHandle n;
	bool success;
	ros::service::waitForService("object_handler/add_object");
	ros::service::waitForService("object_handler/remove_object");
	ros::service::waitForService("object_handler/attach_object");
	ros::service::waitForService("object_handler/detach_object");
	ros::ServiceClient add_client = n.serviceClient<cob_object_handler::HandleObject>("object_handler/add_object");
	ros::ServiceClient remove_client = n.serviceClient<cob_object_handler::HandleObject>("object_handler/remove_object");
	ros::ServiceClient attach_client = n.serviceClient<cob_object_handler::HandleObject>("object_handler/attach_object");
	ros::ServiceClient detach_client = n.serviceClient<cob_object_handler::HandleObject>("object_handler/detach_object");
	cob_object_handler::HandleObject::Request req;
	cob_object_handler::HandleObject::Response res;
	req.object = argv[2];
  
	if(operation=="add")
		success = add_client.call(req, res);
	else if(operation=="remove")
		success = remove_client.call(req, res);
	else if(operation=="attach")
		success = attach_client.call(req, res);
	else if(operation=="detach")
		success = detach_client.call(req, res);
	else
		ROS_ERROR("This operation is not supported");
  
	if (success)
	{
		ROS_INFO("Result: %s", res.error_message.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service");
		return 1;
	}

	return 0;
}
