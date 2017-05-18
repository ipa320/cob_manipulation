#ifndef COB_COLLISION_MONITOR_VALID_STATE_PUBLISHER_H
#define COB_COLLISION_MONITOR_VALID_STATE_PUBLISHER_H

#include <exception>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_msgs/Bool.h>

#include <moveit/collision_detection/collision_common.h>

namespace cob_collision_monitor{
class ValidStatePublisher{
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    bool was_complete_;
    ros::Duration max_age_;
    ros::Publisher pub_;
    std_msgs::Bool data_;
    bool verbose_;
    moveit_msgs::PlanningScene diff_msg_;
public:
    ValidStatePublisher(ros::NodeHandle nh, planning_scene_monitor::PlanningSceneMonitorPtr psm)
    : planning_scene_monitor_(psm),
      was_complete_(false),
      max_age_(nh.param("max_age", 1.0)),
      pub_(nh.advertise<std_msgs::Bool>("state_is_valid", 1)),
      data_(),
      verbose_(nh.param("verbose", true))
    {
        double ground_size = nh.param("ground_size", 10.0);
         
        if(ground_size != 0.0){

            moveit_msgs::CollisionObject co;
            co.header.frame_id = nh.param("ground_link",std::string("base_link"));
            co.id ="_ground_plane_";

            if(ground_size > 0){
                co.primitives.resize(1);
                co.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
                co.primitives.front().dimensions.resize(3);
                co.primitives.front().dimensions[0] = ground_size;
                co.primitives.front().dimensions[1] = ground_size;
                co.primitives.front().dimensions[2] = 0.01;
                co.primitive_poses.resize(1);
                co.primitive_poses.front().position.z = -0.005 + nh.param("ground_height",-0.001);
                co.primitive_poses.front().orientation.w = 1.0;
            }else{
                co.planes.resize(1);
                co.planes.front().coef[2] = 1; // only z, parallel to xy-plane
                co.plane_poses.resize(1);
                co.plane_poses.front().position.z = nh.param("ground_height",-0.001);
                co.plane_poses.front().orientation.w = 1.0;
            }
            diff_msg_.world.collision_objects.push_back(co);
            diff_msg_.is_diff = true;
        }

        data_.data = true;

        planning_scene_monitor_->addUpdateCallback(boost::bind(&ValidStatePublisher::updatedScene,this,_1));
    }


    void updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type){
        try
        {
            planning_scene_monitor::CurrentStateMonitorPtr csm = planning_scene_monitor_->getStateMonitor();
            bool complete = max_age_.isZero() ? csm->haveCompleteState() : csm->haveCompleteState(max_age_);
            if(complete){
                was_complete_ = true;
                planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
                planning_scene::PlanningScenePtr diff_ps = ps->diff(diff_msg_);
                data_.data = !diff_ps->isStateColliding("", verbose_);
                
                /// debug
                collision_detection::CollisionResult::ContactMap contacts;
                diff_ps->getCollidingPairs(contacts);
                ROS_DEBUG("#Collisions: %zu", contacts.size());
                for (collision_detection::CollisionResult::ContactMap::iterator map_it=contacts.begin(); map_it!=contacts.end(); ++map_it)
                {
                    ROS_ERROR("Collision between %s and %s", map_it->first.first.c_str(), map_it->first.second.c_str());
                    ROS_DEBUG("#Contacts: %zu", map_it->second.size());
                    for (std::vector<collision_detection::Contact>::iterator vec_it=map_it->second.begin(); vec_it!=map_it->second.end(); ++vec_it)
                    {
                        ROS_DEBUG("Depth: %f", vec_it->depth);
                    }
                }
            }else if(was_complete_){
                ROS_DEBUG("was not complete");
                data_.data = false;
            }
            pub_.publish(data_);
        }
        catch(std::exception &ex)
        {
            ROS_DEBUG("std::exception: %s", ex.what());
            return;
        }
        catch (...) {
            ROS_DEBUG("Exception occurred");
            return;
        }
    }
};
}

#endif
