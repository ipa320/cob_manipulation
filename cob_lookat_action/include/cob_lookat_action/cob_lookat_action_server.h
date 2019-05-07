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


#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

#include <cob_lookat_action/LookAtAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>



class CobLookAtAction
{
protected:

    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *fjt_ac_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *mbl_ac_;
    actionlib::SimpleActionServer<cob_lookat_action::LookAtAction> *lookat_as_;
    std::string fjt_name_;
    std::string mbl_name_;
    std::string lookat_name_;
    cob_lookat_action::LookAtFeedback lookat_fb_;
    cob_lookat_action::LookAtResult lookat_res_;

    std::vector<std::string> joint_names_;
    std::string chain_base_link_;
    std::string chain_tip_link_;

    KDL::Chain chain_main_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_main_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_pos_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Duration buffer_duration_;

public:

    CobLookAtAction(std::string action_name) :
        fjt_name_("joint_trajectory_controller/follow_joint_trajectory"),
        mbl_name_("/docker_control/move_base_linear"),
        lookat_name_(action_name)
        {}

    ~CobLookAtAction(void) {}

    bool init();
    void goalCB(const cob_lookat_action::LookAtGoalConstPtr &goal);
};
