/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_apps
 * \note
 *   ROS package name: cob_object_handler
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: April 2011
 *
 * \brief
 *   This package provides services for handling a graspable object. 
 *   It reads data from the parameter_server in order to add it to or remove it from the environment_server as a known obstacle.
 * 	 Also it can attach such object to the robot in order to consider it as a part of the robot during the planning process.
 *
 ****************************************************************/



#ifndef OBJECT_HANDLER_INCLUDED
#define OBJECT_HANDLER_INCLUDED


#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>

#include <cob_object_handler/HandleObject.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <cob_object_detection_msgs/DetectObjects.h>

#include "geometric_shapes/shape_operations.h"
#include <planning_environment/util/construct_object.h>

#include <urdf/model.h>


static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string DETECT_OBJECTS_NAME = "/detect_object";


class Object_Handler
{
private:
	ros::NodeHandle rh;

	ros::Publisher m_object_pub;
	ros::Publisher m_att_object_pub;

	ros::ServiceClient m_get_planning_scene_client;
	ros::ServiceClient m_set_planning_scene_diff_client;
	ros::ServiceClient m_detect_objects_client;
	
	ros::ServiceServer m_handle_object_server;
	

public:	
	Object_Handler()
	{
		while(!ros::service::waitForService(GET_PLANNING_SCENE_NAME, ros::Duration(1.0)))
			ROS_WARN("waiting for %s",GET_PLANNING_SCENE_NAME.c_str());
		while(!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.0)))
			ROS_WARN("waiting for %s",SET_PLANNING_SCENE_DIFF_NAME.c_str());
		while(!ros::service::waitForService(DETECT_OBJECTS_NAME, ros::Duration(1.0)))
			ROS_WARN("waiting for %s",DETECT_OBJECTS_NAME.c_str());
		
		m_object_pub  = rh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 1);
		m_att_object_pub  = rh.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 1);
		
		//this sleep is important!
		ros::Duration(0.5).sleep();
		
		m_get_planning_scene_client = rh.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
		m_set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
		m_detect_objects_client = rh.serviceClient<cob_object_detection_msgs::DetectObjects>(DETECT_OBJECTS_NAME);
		
		m_handle_object_server = rh.advertiseService("/object_handler/handle_object", &Object_Handler::handle_object, this);
	}
	
	void run()
	{
		ROS_INFO("object_handler ready...");
		ros::spin();
	}
	
private:	
	//implement callbacks here
	
	bool handle_object(cob_object_handler::HandleObject::Request &req,
					   cob_object_handler::HandleObject::Response &res )
	{
		if(req.operation == "add")
			return add_object(req, res);
		else if(req.operation == "remove")
			return remove_object(req,res);
		else if(req.operation == "attach")
			return attach_object(req,res);
		else if(req.operation == "detach")
			return detach_object(req,res);
		else
			ROS_ERROR("Unkown operation!");
		
		return false;
	}
	
	bool add_object(cob_object_handler::HandleObject::Request  &req,
					cob_object_handler::HandleObject::Response &res )
	{
		ROS_DEBUG("add_object-service called!");
		ROS_INFO("Adding object %s ...",req.id.c_str());
		
		if(req.type == cob_object_handler::HandleObject::Request::SPHERE || req.type == cob_object_handler::HandleObject::Request::BOX || req.type == cob_object_handler::HandleObject::Request::CYLINDER)
		{
			arm_navigation_msgs::CollisionObject collision_object;
			collision_object.id = req.id;
			collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
			collision_object.padding=req.padding;
			
			arm_navigation_msgs::Shape shape;
			shape.type = req.type;
			for(unsigned int i=0; i<req.dimensions.size(); i++)
				shape.dimensions.push_back(req.dimensions[i]);
			
			collision_object.header = req.pose.header;
			collision_object.shapes.push_back(shape);
			collision_object.poses.push_back(req.pose.pose);
			
			m_object_pub.publish(collision_object);
			ROS_DEBUG("Object added to environment server!");
			ros::Duration(0.2).sleep();
		}
		else if(req.type == cob_object_handler::HandleObject::Request::MESH)
		{
			arm_navigation_msgs::CollisionObject collision_object;
			collision_object.id = req.id;
			collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
			collision_object.padding=req.padding;
			
			arm_navigation_msgs::Shape shape;
			const tf::Vector3* scale = new tf::Vector3(req.scale.x, req.scale.y, req.scale.z);
			shapes::Shape *shape_object = shapes::createMeshFromFilename(req.filename.c_str(), scale);
			planning_environment::constructObjectMsg(shape_object, shape);
			
			collision_object.header = req.pose.header;
			collision_object.shapes.push_back(shape);
			collision_object.poses.push_back(req.pose.pose);
			
			m_object_pub.publish(collision_object);
			ROS_DEBUG("Object added to environment server!");
			ros::Duration(0.2).sleep();
		}
		else if(req.type == cob_object_handler::HandleObject::Request::URDF)
		{
			/*
			///initialize urdf::Model from parameter_server 
			std::string parameter_name = req.filename;

			if(!rh.hasParam(parameter_name))
			{
				res.success = false;
				res.error_message = "Parameter cannot be found on paramer server!";
				return true;
			}
			
			urdf::Model model;
			if (!model.initParam(parameter_name))
			{
				res.success = false;
				res.error_message = "Failed to parse parameter!";
				return true;
			}
			*/
			
			urdf::Model model;
			if (!model.initFile(req.filename))
			{
				res.success = false;
				res.error_message = "Failed to create urdf model";
				return true;
			}
			
			std::vector< boost::shared_ptr< urdf::Link > > URDF_links;
			model.getLinks(URDF_links);
			std::map< std::string, boost::shared_ptr< urdf::Joint > > URDF_joints = model.joints_;
			std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator joints_it;
			
			tf::Transform model_origin;
			model_origin = tf::Transform(tf::Quaternion(req.pose.pose.orientation.x, req.pose.pose.orientation.y, req.pose.pose.orientation.z, req.pose.pose.orientation.w), tf::Vector3(req.pose.pose.position.x, req.pose.pose.position.y, req.pose.pose.position.z));
			
			joints_it=URDF_joints.begin();
			for(unsigned int i=0; i<URDF_links.size(); i++) 
			{
				urdf::Link current_link = *URDF_links[i];
				ROS_DEBUG("Current Link: %s", current_link.name.c_str());
				if(current_link.collision == NULL)
				{
					ROS_DEBUG("Current link does not have a collision geometry");
					continue;
				}
				
				arm_navigation_msgs::CollisionObject collision_object;
				collision_object.id = req.id;
				collision_object.id.append("["+current_link.name+"]");
				collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
				collision_object.padding=req.padding;
				
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
				
				collision_object.header = req.pose.header;
				
				m_object_pub.publish(collision_object);
				ROS_DEBUG("Object added to environment server!");
				ros::Duration(0.2).sleep();
			}
			
		}
		else if(req.type == cob_object_handler::HandleObject::Request::DETECT)
		{
			arm_navigation_msgs::CollisionObject collision_object;
			collision_object.id = req.id;
			collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
			collision_object.padding=req.padding;
			
			cob_object_detection_msgs::DetectObjects::Request detect_req;
			cob_object_detection_msgs::DetectObjects::Response detect_res;
			
			detect_req.object_name.data = req.id.c_str();
			
			if(!m_detect_objects_client.call(detect_req, detect_res)) 
			{
				ROS_ERROR("/detect_objects failed");
				res.success = false;
				res.error_message = "Service call /detect_object failed";
				return true;
			}
			if(detect_res.object_list.detections.size() < 1)
			{
				ROS_ERROR("object not found");
				res.success = false;
				res.error_message = "Failed to detect object";
				return true;
			}
			
			cob_object_detection_msgs::Detection detection = detect_res.object_list.detections[0];
			
			arm_navigation_msgs::Shape shape;
			shape.type = cob_object_handler::HandleObject::Request::BOX;
			shape.dimensions.push_back(detection.bounding_box_lwh.x);
			shape.dimensions.push_back(detection.bounding_box_lwh.y);
			shape.dimensions.push_back(detection.bounding_box_lwh.z);
			
			collision_object.header = detection.pose.header;
			collision_object.shapes.push_back(shape);
			collision_object.poses.push_back(detection.pose.pose);
			
			m_object_pub.publish(collision_object);
			ROS_DEBUG("Object added to environment server!");
			ros::Duration(0.2).sleep();
		}
		else
		{
			res.success = false;
			res.error_message = "Object type is not supported!";
			return true;
		}
		
		arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
		arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
		
		if(!m_set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
		{
			ROS_ERROR("Can't get planning scene");
		}
		
		res.success = true;
		res.error_message = "Object added to environment server!";
		return true;
	}
	 
	 
	 
	bool remove_object(cob_object_handler::HandleObject::Request  &req,
					   cob_object_handler::HandleObject::Response &res )
	{
		ROS_DEBUG("remove_object-service called!");
		ROS_INFO("Removing object %s ...",req.id.c_str());
		
		std::string object_name = req.id;
		arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
		arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;
		
		if(!m_get_planning_scene_client.call(get_planning_scene_req, get_planning_scene_res)) 
		{
			ROS_ERROR("Can't get planning scene");
			res.success = false;
			res.error_message = "Can't get planning scene";
			return true;
		}
		
		for(unsigned int i=0; i<get_planning_scene_res.planning_scene.collision_objects.size(); i++)
		{
			if(get_planning_scene_res.planning_scene.collision_objects[i].id.compare(0, object_name.size(),object_name) == 0)
			{
				ROS_DEBUG("Found: %s", get_planning_scene_res.planning_scene.collision_objects[i].id.c_str());
				arm_navigation_msgs::CollisionObject collision_object = get_planning_scene_res.planning_scene.collision_objects[i];
				collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
				
				m_object_pub.publish(collision_object);
				ROS_DEBUG("Object removed from environment server!");
				ros::Duration(0.2).sleep();
			}
		}
		
		arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
		arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
		
		if(!m_set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
		{
			ROS_ERROR("Can't get planning scene");
		}

		res.success = true;
		res.error_message = "Object removed from environment server!";
		return true;
	}
	
	
	bool attach_object(cob_object_handler::HandleObject::Request  &req,
					   cob_object_handler::HandleObject::Response &res )
	{
		ROS_DEBUG("attach_object-service called!");
		ROS_INFO("Attaching object %s ...",req.id.c_str());
		
		std::string object_name = req.id;
		arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
		arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;
		
		if(!m_get_planning_scene_client.call(get_planning_scene_req, get_planning_scene_res)) 
		{
			ROS_ERROR("Can't get planning scene");
			res.success = false;
			res.error_message = "Can't get planning scene";
			return true;
		}
		
		for(unsigned int i=0; i<get_planning_scene_res.planning_scene.collision_objects.size(); i++)
		{
			if(get_planning_scene_res.planning_scene.collision_objects[i].id.compare(0, object_name.size(),object_name) == 0)
			{
				arm_navigation_msgs::AttachedCollisionObject att_object;
				att_object.object = get_planning_scene_res.planning_scene.collision_objects[i];
				//attach it to the SDH
				att_object.link_name = req.attach_link;
				for(unsigned int k=0; k<req.touch_links.size(); k++)
					att_object.touch_links.push_back(req.touch_links[i]);
				
				att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
				
				m_att_object_pub.publish(att_object);
				ROS_DEBUG("Object attached to robot! %s", get_planning_scene_res.planning_scene.collision_objects[i].id.c_str());
				ros::Duration(0.2).sleep();
			}
		}
		
		arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
		arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
		
		if(!m_set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
		{
			ROS_ERROR("Can't get planning scene");
		}
		
		
		res.success = true;
		res.error_message = "Object attached to robot!";
		return true;
	}
	
	
	bool detach_object(cob_object_handler::HandleObject::Request  &req,
					   cob_object_handler::HandleObject::Response &res )
	{
		ROS_DEBUG("detach_object-service called!");
		ROS_INFO("Detaching object %s ...",req.id.c_str());
		
		std::string object_name = req.id;
		
		arm_navigation_msgs::GetPlanningScene::Request get_planning_scene_req;
		arm_navigation_msgs::GetPlanningScene::Response get_planning_scene_res;
		
		if(!m_get_planning_scene_client.call(get_planning_scene_req, get_planning_scene_res)) 
		{
			ROS_ERROR("Can't get planning scene");
			res.success = false;
			res.error_message = "Can't get planning scene";
			return true;
		}
		
		for(unsigned int i=0; i<get_planning_scene_res.planning_scene.attached_collision_objects.size(); i++)
		{
			if(get_planning_scene_res.planning_scene.attached_collision_objects[i].object.id.compare(0, object_name.size(),object_name) == 0)
			{
				arm_navigation_msgs::AttachedCollisionObject att_object;
				att_object = get_planning_scene_res.planning_scene.attached_collision_objects[i];
				att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
				
				m_att_object_pub.publish(att_object);
				ROS_DEBUG("Object detached from robot!");
				ros::Duration(0.2).sleep();
			}
		}
		arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
		arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
		
		if(!m_set_planning_scene_diff_client.call(set_planning_scene_diff_req, set_planning_scene_diff_res)) 
		{
			ROS_ERROR("Can't get planning scene");
		}
		
		res.success = true;
		res.error_message = "Object detached from robot!";
		return true;
	}
	
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///          helper functions
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
				const tf::Vector3* scale = new tf::Vector3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
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
	
///NOT USED ANYMORE
/*	
	//these are for parsin *.model files
	bool parse_box(std::string model_parameter, std::vector< double > &dimensions)
	{
		//parsing-scheme only valid for "box" (size: x,y,z)
		std::string pattern;
		std::size_t found_size, found_p, found_x, found_y, found_z;

		pattern = "size";
		found_size=model_parameter.find(pattern);
		if (found_size!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = ">";
		found_p=model_parameter.find(pattern, found_size);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";
		found_x=model_parameter.find_first_not_of(pattern, found_p+1);
		if (found_x!=std::string::npos)
			ROS_DEBUG("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";	
		found_p=model_parameter.find_first_of(pattern, found_x);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_x = found_p - found_x;
		std::string x = model_parameter.substr(found_x, length_x);
		ROS_DEBUG("x: %s, real_length_x: %d", x.c_str(), int(length_x));

		double x_d = strtod(x.c_str(), NULL);
		ROS_INFO("x as double: %f", x_d);



		pattern = " ";
		found_y=model_parameter.find_first_not_of(pattern, found_p);
		if (found_y!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_y));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		found_p=model_parameter.find_first_of(pattern, found_y);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_y = found_p - found_y;
		std::string y = model_parameter.substr(found_y, length_y);
		ROS_DEBUG("y: %s, real_length_y: %d", y.c_str(), int(length_y));

		double y_d = strtod(y.c_str(), NULL);
		ROS_INFO("y as double: %f", y_d);




		pattern = " ";
		found_z=model_parameter.find_first_not_of(pattern, found_p);
		if (found_z!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_z));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = "<";	
		found_p=model_parameter.find_first_of(pattern, found_z);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_z = found_p - found_z;
		std::string z = model_parameter.substr(found_z, length_z);
		ROS_DEBUG("z: %s, real_length_z: %d", z.c_str(), int(length_z));

		double z_d = strtod(z.c_str(), NULL);
		ROS_INFO("z as double: %f", z_d);

		dimensions.push_back(x_d);
		dimensions.push_back(y_d);
		dimensions.push_back(z_d);
			
		return true;
	}
	
	
	bool compose_box(std::string model_name, std::vector< double > dimensions, arm_navigation_msgs::CollisionObject &collision_object)
	{
		
		gazebo::GetModelState state_srv;

		state_srv.request.model_name = model_name;
		if (m_state_client.call(state_srv))
		{
			ROS_INFO("ModelPose (x,y,z): (%f,%f,%f)", state_srv.response.pose.position.x, state_srv.response.pose.position.y, state_srv.response.pose.position.z);
		}
		else
		{
			ROS_ERROR("Failed to call service get_model_state");
			return false;
		}

		collision_object.id = model_name;
		//collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
		//collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
		collision_object.header.frame_id = frame_id;
		collision_object.header.stamp = ros::Time::now();
		collision_object.shapes.resize(1);
		collision_object.poses.resize(1);

		//ToDo: figure out how *.model-size and *.urdf-extend are related
		//ToDo: figure out where the *.model origin is located (top,center,bottom?)
		collision_object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
		collision_object.shapes[0].dimensions.push_back(dimensions[0]/2.0);
		collision_object.shapes[0].dimensions.push_back(dimensions[1]/2.0);
		collision_object.shapes[0].dimensions.push_back(dimensions[2]/2.0);

		collision_object.poses[0].position.x = state_srv.response.pose.position.x;
		collision_object.poses[0].position.y = state_srv.response.pose.position.y;
		collision_object.poses[0].position.z = state_srv.response.pose.position.z;
		collision_object.poses[0].orientation.x = state_srv.response.pose.orientation.x;
		collision_object.poses[0].orientation.y = state_srv.response.pose.orientation.y;
		collision_object.poses[0].orientation.z = state_srv.response.pose.orientation.z;
		collision_object.poses[0].orientation.w = state_srv.response.pose.orientation.w;
		
		return true;
	}
*/
};

#endif
