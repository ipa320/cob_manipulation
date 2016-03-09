#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_obstacle_publisher_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);
  ros::Duration(1.0).sleep();

  moveit_msgs::CollisionObject msg;
  msg.id = "test_objects";
  msg.header.frame_id = "odom_combined";
  msg.operation = moveit_msgs::CollisionObject::ADD;

  // primitive sphere
  shape_msgs::SolidPrimitive sphere;
  sphere.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere.dimensions.push_back(0.1);

  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  msg.primitives.push_back(sphere);
  msg.primitive_poses.push_back(pose);

  // mesh sphere
  shapes::Mesh* sphere_shape = shapes::createMeshFromResource("package://obstacle_distance/files/sphere.stl");
  shapes::ShapeMsg sphere_mesh_msg;
  shapes::constructMsgFromShape(sphere_shape, sphere_mesh_msg);
  shape_msgs::Mesh sphere_mesh = boost::get<shape_msgs::Mesh>(sphere_mesh_msg);

  pose.position.x = 1.0;
  pose.position.y = 1.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  msg.meshes.push_back(sphere_mesh);
  msg.mesh_poses.push_back(pose);

  // primitive box
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.push_back(0.1);
  box.dimensions.push_back(0.1);
  box.dimensions.push_back(0.1);

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;

  msg.primitives.push_back(box);
  msg.primitive_poses.push_back(pose);

  // mesh box
  shapes::Mesh* box_shape = shapes::createMeshFromResource("package://obstacle_distance/files/box.stl");
  shapes::ShapeMsg box_mesh_msg;
  shapes::constructMsgFromShape(box_shape, box_mesh_msg);
  shape_msgs::Mesh box_mesh = boost::get<shape_msgs::Mesh>(box_mesh_msg);

  pose.position.x = 1.0;
  pose.position.y = 1.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;

  msg.meshes.push_back(box_mesh);
  msg.mesh_poses.push_back(pose);

  // primitive cylinder
  shape_msgs::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder.dimensions.push_back(0.1);
  cylinder.dimensions.push_back(0.1);

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 2.0;
  pose.orientation.w = 1.0;

  msg.primitives.push_back(cylinder);
  msg.primitive_poses.push_back(pose);

  // mesh cylinder
  shapes::Mesh* cylinder_shape = shapes::createMeshFromResource("package://obstacle_distance/files/cylinder.stl");
  shapes::ShapeMsg cylinder_mesh_msg;
  shapes::constructMsgFromShape(cylinder_shape, cylinder_mesh_msg);
  shape_msgs::Mesh cylinder_mesh = boost::get<shape_msgs::Mesh>(cylinder_mesh_msg);

  pose.position.x = 1.0;
  pose.position.y = 1.0;
  pose.position.z = 2.0;
  pose.orientation.w = 1.0;

  msg.meshes.push_back(cylinder_mesh);
  msg.mesh_poses.push_back(pose);




  pub.publish(msg);
  ros::Duration(2.0).sleep();
  ROS_INFO("Done");

  return 0;
}

