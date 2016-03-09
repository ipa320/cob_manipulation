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

  // primitive sphere
  moveit_msgs::CollisionObject col_sphere_msg;
  col_sphere_msg.id = "sphere_primitive";
  col_sphere_msg.header.frame_id = "odom_combined";
  col_sphere_msg.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive sphere;
  sphere.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere.dimensions.push_back(0.1);

  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  col_sphere_msg.primitives.push_back(sphere);
  col_sphere_msg.primitive_poses.push_back(pose);

  pub.publish(col_sphere_msg);
  ros::Duration(0.1).sleep();

  // mesh sphere
  moveit_msgs::CollisionObject col_sphere_mesh_msg;
  col_sphere_mesh_msg.id = "sphere_mesh";
  col_sphere_mesh_msg.header.frame_id = "odom_combined";
  col_sphere_mesh_msg.operation = moveit_msgs::CollisionObject::ADD;

  shapes::Mesh* sphere_shape = shapes::createMeshFromResource("package://obstacle_distance/files/sphere.stl");
  shapes::ShapeMsg sphere_mesh_msg;
  shapes::constructMsgFromShape(sphere_shape, sphere_mesh_msg);
  shape_msgs::Mesh sphere_mesh = boost::get<shape_msgs::Mesh>(sphere_mesh_msg);

  pose.position.x = 1.0;
  // pose.position.y = 1.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  col_sphere_mesh_msg.meshes.push_back(sphere_mesh);
  col_sphere_mesh_msg.mesh_poses.push_back(pose);

  pub.publish(col_sphere_mesh_msg);
  ros::Duration(0.1).sleep();

  // primitive box
  moveit_msgs::CollisionObject col_box_msg;
  col_box_msg.id = "box_primitive";
  col_box_msg.header.frame_id = "odom_combined";
  col_box_msg.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.push_back(0.1);
  box.dimensions.push_back(0.1);
  box.dimensions.push_back(0.1);

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;

  col_box_msg.primitives.push_back(box);
  col_box_msg.primitive_poses.push_back(pose);

  pub.publish(col_box_msg);
  ros::Duration(0.1).sleep();

  // mesh box
  moveit_msgs::CollisionObject col_box_mesh_msg;
  col_box_mesh_msg.id = "box_mesh";
  col_box_mesh_msg.header.frame_id = "odom_combined";
  col_box_mesh_msg.operation = moveit_msgs::CollisionObject::ADD;

  shapes::Mesh* box_shape = shapes::createMeshFromResource("package://obstacle_distance/files/box.stl");
  shapes::ShapeMsg box_mesh_msg;
  shapes::constructMsgFromShape(box_shape, box_mesh_msg);
  shape_msgs::Mesh box_mesh = boost::get<shape_msgs::Mesh>(box_mesh_msg);

  pose.position.x = 1.0;
  // pose.position.y = 1.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;

  col_box_mesh_msg.meshes.push_back(box_mesh);
  col_box_mesh_msg.mesh_poses.push_back(pose);

  pub.publish(col_box_mesh_msg);
  ros::Duration(0.1).sleep();

  // primitive cylinder
  moveit_msgs::CollisionObject col_cylinder_msg;
  col_cylinder_msg.id = "cylinder_primitive";
  col_cylinder_msg.header.frame_id = "odom_combined";
  col_cylinder_msg.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder.dimensions.push_back(0.1);
  cylinder.dimensions.push_back(0.1);

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 2.0;
  pose.orientation.w = 1.0;

  col_cylinder_msg.primitives.push_back(cylinder);
  col_cylinder_msg.primitive_poses.push_back(pose);

  pub.publish(col_cylinder_msg);
  ros::Duration(0.1).sleep();

  // mesh cylinder
  moveit_msgs::CollisionObject col_cylinder_mesh_msg;
  col_cylinder_mesh_msg.id = "cylinder_mesh";
  col_cylinder_mesh_msg.header.frame_id = "odom_combined";
  col_cylinder_mesh_msg.operation = moveit_msgs::CollisionObject::ADD;

  shapes::Mesh* cylinder_shape = shapes::createMeshFromResource("package://obstacle_distance/files/cylinder.stl");
  shapes::ShapeMsg cylinder_mesh_msg;
  shapes::constructMsgFromShape(cylinder_shape, cylinder_mesh_msg);
  shape_msgs::Mesh cylinder_mesh = boost::get<shape_msgs::Mesh>(cylinder_mesh_msg);

  pose.position.x = 1.0;
  // pose.position.y = 1.0;
  pose.position.z = 2.0;
  pose.orientation.w = 1.0;

  col_cylinder_mesh_msg.meshes.push_back(cylinder_mesh);
  col_cylinder_mesh_msg.mesh_poses.push_back(pose);

  pub.publish(col_cylinder_mesh_msg);
  ros::Duration(0.1).sleep();






  /// test robot objects
  // primitive sphere
  moveit_msgs::CollisionObject test_col_sphere_msg;
  test_col_sphere_msg.id = "test_primitive";
  test_col_sphere_msg.header.frame_id = "odom_combined";
  test_col_sphere_msg.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive test_sphere;
  test_sphere.type = shape_msgs::SolidPrimitive::SPHERE;
  test_sphere.dimensions.push_back(0.1);

  geometry_msgs::Pose test_pose;
  test_pose.position.x = 2.0;
  test_pose.position.y = 0.0;
  test_pose.position.z = 1.0;
  test_pose.orientation.w = 1.0;

  test_col_sphere_msg.primitives.push_back(test_sphere);
  test_col_sphere_msg.primitive_poses.push_back(test_pose);

  pub.publish(test_col_sphere_msg);
  ros::Duration(0.1).sleep();

  // mesh sphere
  moveit_msgs::CollisionObject test_col_sphere_mesh_msg;
  test_col_sphere_mesh_msg.id = "test_mesh";
  test_col_sphere_mesh_msg.header.frame_id = "odom_combined";
  test_col_sphere_mesh_msg.operation = moveit_msgs::CollisionObject::ADD;

  shapes::Mesh* test_sphere_shape = shapes::createMeshFromResource("package://obstacle_distance/files/sphere.stl");
  shapes::ShapeMsg test_sphere_mesh_msg;
  shapes::constructMsgFromShape(test_sphere_shape, test_sphere_mesh_msg);
  shape_msgs::Mesh test_sphere_mesh = boost::get<shape_msgs::Mesh>(test_sphere_mesh_msg);

  test_pose.position.x = 2.0;
  // test_pose.position.y = 1.0;
  test_pose.position.z = 1.0;
  test_pose.orientation.w = 1.0;

  test_col_sphere_mesh_msg.meshes.push_back(test_sphere_mesh);
  test_col_sphere_mesh_msg.mesh_poses.push_back(test_pose);

  pub.publish(test_col_sphere_mesh_msg);
  ros::Duration(0.1).sleep();



  ROS_INFO("Done");

  return 0;
}

