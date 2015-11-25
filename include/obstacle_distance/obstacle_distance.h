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

class ObstacleDistance : public ros::NodeHandle {
public:
    ObstacleDistance();

private:
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    float MAXIMAL_MINIMAL_DISTANCE;
    ros::Publisher obstacle_distance_publisher_;

    void updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);

};

#endif //OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
