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
