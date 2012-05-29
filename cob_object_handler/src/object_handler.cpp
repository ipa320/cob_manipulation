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



#include "ros/ros.h"
#include <cob_object_handler/object_handler.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_object_handler_node");
	
	Object_Handler* object_handler = new Object_Handler();

	object_handler->run();

	return 0;
}


