#include <moveit/move_group/move_group_capability.h>
#include "valid_state_publisher.h"

namespace cob_collision_monitor{
class CollisionMonitor : public move_group::MoveGroupCapability
{
    boost::scoped_ptr<ValidStatePublisher> vsp_;
    virtual void initialize(){
        vsp_.reset(new ValidStatePublisher(node_handle_, context_->planning_scene_monitor_));
    }
public:
    CollisionMonitor(): MoveGroupCapability("CollisionMonitor")
    {
    }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cob_collision_monitor::CollisionMonitor, move_group::MoveGroupCapability)