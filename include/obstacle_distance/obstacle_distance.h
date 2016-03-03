#ifndef OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
#define OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H

#include <ros/ros.h>

#include <string.h>
#include <map>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <cob_srvs/SetString.h>
#include <obstacle_distance/GetObstacleDistance.h>
#include <obstacle_distance/DistanceInfos.h>

#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <kdl_conversions/kdl_msg.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <urdf/model.h>


class ObstacleDistance : public ros::NodeHandle {
public:
    ObstacleDistance();

private:
    ros::NodeHandle nh_;
    
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    float MAXIMAL_MINIMAL_DISTANCE;
    ros::ServiceServer calculate_obstacle_distance_;

    ros::ServiceServer register_server_, unregister_server_;
    obstacle_distance::DistanceInfos distance_infos_;
    std::set< std::string > registered_links_;
    boost::mutex registered_links_mutex_;

    ros::Publisher distance_pub_;
    ros::Timer distance_timer_, planning_scene_timer_;

    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links_list;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects_list;
    std::vector<std::string> kinematic_list;

    void updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);
    ros::ServiceServer monitored_scene_server_;
    ros::Publisher monitored_scene_pub_;
    urdf::Model model_;
    typedef boost::shared_ptr<const urdf::Link> PtrConstLink_t;

    bool calculateDistanceCallback(obstacle_distance::GetObstacleDistance::Request &req,
                                   obstacle_distance::GetObstacleDistance::Response &res);

    bool registerCallback(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res);
    bool unregisterCallback(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res);
    bool planningSceneCallback(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res);
    void planningSceneTimerCallback(const ros::TimerEvent& event);

    void calculateDistances(const ros::TimerEvent& event);

    obstacle_distance::DistanceInfo getDistanceInfo(const boost::shared_ptr<fcl::CollisionObject> robot_link,
                                                    const boost::shared_ptr<fcl::CollisionObject> collision_object,
                                                    bool do_transform_robot_link, bool do_transform_selfcollision_object);
    bool getRootFrame(const boost::shared_ptr<fcl::CollisionObject> co);
};

#endif //OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
