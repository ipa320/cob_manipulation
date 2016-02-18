#include "valid_state_publisher.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "collision_monitor_node");

    ros::NodeHandle nh("~");

    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(nh.param("max_cache_time", 2.0))));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));

    double state_update_frequency;
    if(nh.getParam("state_update_frequency", state_update_frequency))
        psm->setStateUpdateFrequency(state_update_frequency);

    if(nh.param("start_scene_monitor", true))
        psm->startSceneMonitor();
    if(nh.param("start_world_geometry_monitor", true))
        psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    cob_collision_monitor::ValidStatePublisher vsp(nh, psm);

    ros::spin();

    return 0;
}
