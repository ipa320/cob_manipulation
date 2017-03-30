#ifndef COB_OBSTACLE_DISTANCE_MOVEIT__OBSTACLE_DISTANCE_H
#define COB_OBSTACLE_DISTANCE_MOVEIT__OBSTACLE_DISTANCE_H

#include <map>
#include <string.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <cob_srvs/SetString.h>
#include <cob_control_msgs/ObstacleDistances.h>
#include <cob_control_msgs/GetObstacleDistance.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

class ObstacleDistanceMoveit
{
public:
    ObstacleDistanceMoveit();

private:
    ros::NodeHandle nh_;    
    float MAXIMAL_MINIMAL_DISTANCE;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    void updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);

    ros::Timer planning_scene_timer_;
    ros::Publisher monitored_scene_pub_;
    ros::ServiceServer monitored_scene_server_;
    bool planningSceneCallback(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res);
    void planningSceneTimerCallback(const ros::TimerEvent& event);

    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links_;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects_;
    std::set< std::string > registered_links_;
    boost::mutex registered_links_mutex_;

    ros::ServiceServer calculate_obstacle_distance_;
    bool calculateDistanceServiceCallback(cob_control_msgs::GetObstacleDistance::Request &req,
                                          cob_control_msgs::GetObstacleDistance::Response &res);

    ros::Publisher distance_pub_;
    ros::ServiceServer register_server_, unregister_server_;
    bool registerCallback(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res);
    bool unregisterCallback(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res);

    ros::Timer distance_timer_;
    void calculateDistanceTimerCallback(const ros::TimerEvent& event);

    cob_control_msgs::ObstacleDistance getDistanceInfo(const boost::shared_ptr<fcl::CollisionObject> object_a,
                                                       const boost::shared_ptr<fcl::CollisionObject> object_b);

    collision_detection::AllowedCollisionMatrix acm_;
};

#endif  // COB_OBSTACLE_DISTANCE_MOVEIT__OBSTACLE_DISTANCE_H
