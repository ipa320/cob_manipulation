#ifndef OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
#define OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

#include <boost/thread.hpp>

#include <sensor_msgs/JointState.h>
#include <atf_msgs/ObstacleDistance.h>
#include <atf_msgs/ObstacleDistanceLink.h>
#include <obstacle_distance/GetObstacleDistance.h>

class ObstacleDistance : public ros::NodeHandle {
public:
    ObstacleDistance();

private:
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    float MAXIMAL_MINIMAL_DISTANCE;
    ros::ServiceServer calculate_obstacle_distance_;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links_list;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects_list;
    std::vector<std::string> kinematic_list;

    void updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);
    bool calculateDistance(obstacle_distance::GetObstacleDistance::Request &req,
                           obstacle_distance::GetObstacleDistance::Response &res);

    double getMinimalDistance(std::string robot_link_name,
                              std::string collision_object_name,
                              std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links,
                              std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects);

};

#endif //OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
