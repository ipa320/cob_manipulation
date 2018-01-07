/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
