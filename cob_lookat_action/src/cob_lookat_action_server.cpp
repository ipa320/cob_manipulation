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
#include <ros/ros.h>
#include <angles/angles.h>
#include <cob_lookat_action/cob_lookat_action_server.h>


bool CobLookAtAction::init()
{
    /// get parameters from parameter server
    if (!nh_.getParam("joint_names", joint_names_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    if (!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    /// parse robot_description and generate KDL chains
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("/robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    tree.getChain(chain_base_link_, chain_tip_link_, chain_main_);
    if (chain_main_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }


    ROS_WARN_STREAM("Waiting for ActionServer: " << fjt_name_);
    fjt_ac_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, fjt_name_, true);
    fjt_ac_->waitForServer(ros::Duration(10.0));

    lookat_as_ = new actionlib::SimpleActionServer<cob_lookat_action::LookAtAction>(nh_, lookat_name_, boost::bind(&CobLookAtAction::goalCB, this, _1), false);
    lookat_as_->start();

    return true;
}


void CobLookAtAction::goalCB(const cob_lookat_action::LookAtGoalConstPtr &goal)
{
    bool success = true;
    std::string message;
    double roll, pitch, yaw;

    /// transform pointing_frame to offset
    KDL::Frame offset;
    tf::StampedTransform offset_transform;
    bool transformed = false;

    do
    {
        try
        {
            tf_listener_.lookupTransform(chain_tip_link_, goal->pointing_frame, ros::Time(0), offset_transform);
            transformed = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("LookatAction: %s", ex.what());
            ros::Duration(0.1).sleep();
        }
    } while (!transformed && ros::ok() && !lookat_as_->isPreemptRequested());

    if (lookat_as_->isPreemptRequested() )
    {
        success = false;
        message = "Preempted after lookup 'pointing_frame'";
        ROS_WARN_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setPreempted(lookat_res_);
        return;
    }

    tf::transformTFToKDL(offset_transform, offset);

    //ToDo: implement offset mechanism
    //tf::transformMsgToKDL(goal->pointing_offset, offset);

    offset.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("offset.p: " << offset.p.x() << ", " << offset.p.y() << ", " << offset.p.z());
    ROS_WARN_STREAM("offset.rot: " << roll << ", " << pitch << ", " << yaw);

    /// compose lookat_lin joint
    KDL::Chain chain_base, chain_lookat, chain_full;
    KDL::Vector lookat_lin_axis(0.0, 0.0, 0.0);
    switch (goal->pointing_axis_type)
    {
        case cob_lookat_action::LookAtGoal::X_POSITIVE:
            lookat_lin_axis.x(1.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_POSITIVE:
            lookat_lin_axis.y(1.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_POSITIVE:
            lookat_lin_axis.z(1.0);
            break;
        case cob_lookat_action::LookAtGoal::X_NEGATIVE:
            lookat_lin_axis.x(-1.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_NEGATIVE:
            lookat_lin_axis.y(-1.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_NEGATIVE:
            lookat_lin_axis.z(-1.0);
            break;
        default:
            ROS_ERROR("PointingAxisType %d not defined! Using default: 'X_POSITIVE'!", goal->pointing_axis_type);
            lookat_lin_axis.x(1.0);
            break;
    }
    ROS_WARN("PointingAxisType %d: %f, %f, %f", goal->pointing_axis_type, lookat_lin_axis.x(), lookat_lin_axis.y(), lookat_lin_axis.z());

    //ToDo: fix negative axes

    //fixed pointing offset
    KDL::Joint offset_joint("offset_joint", KDL::Joint::None);
    KDL::Segment offset_link("offset_link", offset_joint, offset);
    chain_lookat.addSegment(offset_link);

    //chain_lookat
    KDL::Joint lookat_lin_joint("lookat_lin_joint", KDL::Vector(), lookat_lin_axis, KDL::Joint::TransAxis);
    KDL::Segment lookat_rotx_link("lookat_rotx_link", lookat_lin_joint);
    chain_lookat.addSegment(lookat_rotx_link);

    KDL::Joint lookat_rotx_joint("lookat_rotx_joint", KDL::Joint::RotX);
    KDL::Segment lookat_roty_link("lookat_roty_link", lookat_rotx_joint);
    chain_lookat.addSegment(lookat_roty_link);

    KDL::Joint lookat_roty_joint("lookat_roty_joint", KDL::Joint::RotY);
    KDL::Segment lookat_rotz_link("lookat_rotz_link", lookat_roty_joint);
    chain_lookat.addSegment(lookat_rotz_link);

    KDL::Joint lookat_rotz_joint("lookat_rotz_joint", KDL::Joint::RotZ);
    KDL::Segment lookat_focus_frame("lookat_focus_frame", lookat_rotz_joint);
    chain_lookat.addSegment(lookat_focus_frame);

    //chain_base
    KDL::Joint base_rotz_joint("base_rotz_joint", KDL::Joint::RotZ);
    KDL::Segment base_rotz_link("base_rotz_link", base_rotz_joint, KDL::Frame());
    chain_base.addSegment(base_rotz_link);

    //chain composition
    if ( goal->base_active )
    {
        chain_full = chain_base;
        chain_full.addChain(chain_main_);
    }
    else
    {
        chain_full = chain_main_;
    }
    chain_full.addChain(chain_lookat);

    /// set up solver
    fk_solver_pos_main_.reset(new KDL::ChainFkSolverPos_recursive(chain_main_));
    fk_solver_pos_.reset(new KDL::ChainFkSolverPos_recursive(chain_full));
    ik_solver_pos_.reset(new KDL::ChainIkSolverPos_LMA(chain_full));
    //ToDo: test other solvers
    //ChainIkSolverPos_NR_NL: no
    //ChainIkSolverPos_NR w. KDL::ChainIkSolverVel_pinv: no
    //ChainIkSolverPos_NR w. KDL::ChainIkSolverVel_pinv_givens: ?
    //ChainIkSolverPos_NR w. KDL::ChainIkSolverVel_pinv_nso: ?
    //ChainIkSolverPos_NR w. KDL::ChainIkSolverVel_wdls: ?

    /// transform target_frame to p_in
    KDL::Frame p_in;
    tf::StampedTransform transform_in;
    transformed = false;

    do
    {
        try
        {
            tf_listener_.lookupTransform(chain_base_link_, goal->target_frame, ros::Time(0), transform_in);
            transformed = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("LookatAction: %s", ex.what());
            ros::Duration(0.1).sleep();
        }
    } while (!transformed && ros::ok() && !lookat_as_->isPreemptRequested());

    if (lookat_as_->isPreemptRequested() )
    {
        success = false;
        message = "Preempted after lookup 'target_frame'";
        ROS_WARN_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setPreempted(lookat_res_);
        return;
    }

    //ToDo: check age of transformation
    ros::Time now = ros::Time::now();
    ROS_INFO_STREAM("NOW: " << now << ", STAMP: " << transform_in.stamp_ << ", DIFF: " << (now - transform_in.stamp_).toSec());

    //ToDo: "upright"-constraint

    tf::transformTFToKDL(transform_in, p_in);
    KDL::JntArray q_init(chain_full.getNrOfJoints());
    KDL::JntArray q_out(chain_full.getNrOfJoints());

    p_in.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("p_in.p: " << p_in.p.x() << ", " << p_in.p.y() << ", " << p_in.p.z());
    ROS_WARN_STREAM("p_in.rot: " << roll << ", " << pitch << ", " << yaw);

    int result_ik = ik_solver_pos_->CartToJnt(q_init, p_in, q_out);

    ROS_WARN_STREAM("IK-Error: "<< ik_solver_pos_->getError() << " - " << ik_solver_pos_->strError(ik_solver_pos_->getError()));
    ROS_WARN_STREAM("q_init: " << q_init.data);
    ROS_WARN_STREAM("q_out: " << q_out.data);

    /// solution valid?
    if (ik_solver_pos_->getError() != KDL::SolverI::E_NOERROR)
    {
        success = false;
        message = "Failed to find IK solution";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    //ToDo. check joint_limits

    /// check FK diff (target vs. q_out)
    KDL::Frame p_out;
    int result_fk = fk_solver_pos_->JntToCart(q_out, p_out);

    ROS_WARN_STREAM("FK-Error: "<< fk_solver_pos_->getError() << " - " << fk_solver_pos_->strError(fk_solver_pos_->getError()));
    p_out.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("p_out.p: " << p_out.p.x() << ", " << p_out.p.y() << ", " << p_out.p.z());
    ROS_WARN_STREAM("p_out.rot: " << roll << ", " << pitch << ", " << yaw);

    /// solution valid?
    if (fk_solver_pos_->getError() != KDL::SolverI::E_NOERROR)
    {
        success = false;
        message = "Failed to find FK solution";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    KDL::Vector v_diff = KDL::diff(p_in.p, p_out.p);
    ROS_WARN_STREAM("p_diff: " << v_diff.x() << ", " << v_diff.y() << ", " << v_diff.z());
    ROS_WARN_STREAM("NORM v_diff: " << v_diff.Norm());

    if (!KDL::Equal(p_in, p_out, 1.0) )
    {
        ROS_WARN_STREAM("P_IN !Equal P_OUT");
    }

    /// check FK based on main + offset
    KDL::Frame p_out_main;
    KDL::JntArray q_out_main(chain_main_.getNrOfJoints());
    unsigned int k;
    if ( goal->base_active ) { k=1; }
    else { k=0; }
    for(unsigned int i = 0; i < chain_main_.getNrOfJoints(); i++)
    {
        q_out_main(i)=angles::normalize_angle(q_out(i+k));
    }
    int result_fk_main = fk_solver_pos_main_->JntToCart(q_out_main, p_out_main);
    KDL::Frame p_out_main_offset = p_out_main*offset;
    KDL::Frame tip2target = p_out_main_offset.Inverse()*p_in;

    ROS_WARN_STREAM("FK-Error MAIN: "<< fk_solver_pos_main_->getError() << " - " << fk_solver_pos_main_->strError(fk_solver_pos_main_->getError()));

    ROS_WARN_STREAM("q_out_main: " << q_out_main.data);
    p_out_main.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("p_out_main.p: " << p_out_main.p.x() << ", " << p_out_main.p.y() << ", " << p_out_main.p.z());
    ROS_WARN_STREAM("p_out_main.rot: " << roll << ", " << pitch << ", " << yaw);
    offset.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("offset.p: " << offset.p.x() << ", " << offset.p.y() << ", " << offset.p.z());
    ROS_WARN_STREAM("offset.rot: " << roll << ", " << pitch << ", " << yaw);
    p_out_main_offset.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("p_out_main_offset.p: " << p_out_main_offset.p.x() << ", " << p_out_main_offset.p.y() << ", " << p_out_main_offset.p.z());
    ROS_WARN_STREAM("p_out_main_offset.rot: " << roll << ", " << pitch << ", " << yaw);
    tip2target.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("tip2target.p: " << tip2target.p.x() << ", " << tip2target.p.y() << ", " << tip2target.p.z());
    ROS_WARN_STREAM("tip2target.rot: " << roll << ", " << pitch << ", " << yaw);

    /// solution valid?
    if (fk_solver_pos_main_->getError() != KDL::SolverI::E_NOERROR)
    {
        success = false;
        message = "Failed to find FK solution MAIN";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    //ToDo: check lookat conformity
    KDL::Frame tip2target_test(tip2target);
    double q_lookat_lin = 0.0;
    switch (goal->pointing_axis_type)
    {
        case cob_lookat_action::LookAtGoal::X_POSITIVE:
            q_lookat_lin = tip2target.p.x();
            tip2target_test.p.x(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_POSITIVE:
            q_lookat_lin = tip2target.p.y();
            tip2target_test.p.y(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_POSITIVE:
            q_lookat_lin = tip2target.p.z();
            tip2target_test.p.z(0.0);
            break;
        case cob_lookat_action::LookAtGoal::X_NEGATIVE:
            q_lookat_lin = tip2target.p.x();
            tip2target_test.p.x(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_NEGATIVE:
            q_lookat_lin = tip2target.p.y();
            tip2target_test.p.y(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_NEGATIVE:
            q_lookat_lin = tip2target.p.z();
            tip2target_test.p.z(0.0);
            break;
        default:
            ROS_ERROR("PointingAxisType %d not defined! Using default: 'X_POSITIVE'!", goal->pointing_axis_type);
            q_lookat_lin = tip2target.p.x();
            tip2target_test.p.x(0.0);
            break;
    }

    /// check lin_axis value positive
    ROS_WARN_STREAM("q_lookat_lin: " << q_lookat_lin);
    if ( q_lookat_lin < 0.0 )
    {
        success = false;
        message = "q_lookat_lin is negative";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    /// check lookat offset from lin_axis
    ROS_WARN_STREAM("tip2target_test: " << tip2target_test.p.x() << ", " << tip2target_test.p.y() << ", " << tip2target_test.p.z());
    ROS_WARN_STREAM("tip2target_test.p.Norm: " << tip2target_test.p.Norm());
    if ( tip2target_test.p.Norm() > 0.1 )
    {
        success = false;
        message = "Tip2Target is not lookat-conform";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    if ( goal->base_active )
    {
        ROS_ERROR_STREAM(lookat_name_ << ": MOVE_BASE_REL: " << angles::normalize_angle(q_out(0)));
    }

    /// execute solution as FJT
    control_msgs::FollowJointTrajectoryGoal fjt_goal;
    fjt_goal.trajectory.header.stamp = ros::Time::now();
    fjt_goal.trajectory.header.frame_id = chain_base_link_;
    fjt_goal.trajectory.joint_names = joint_names_;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    for(unsigned int i = 0; i < chain_main_.getNrOfJoints(); i++)
    {
        traj_point.positions.push_back(angles::normalize_angle(q_out(i+k)));
    }
    traj_point.time_from_start = ros::Duration(3.0);
    fjt_goal.trajectory.points.push_back(traj_point);

    /// disable FJT constraints
    //There are two special values for tolerances:
    // *  0 - The tolerance is unspecified and will remain at whatever the default is
    // * -1 - The tolerance is "erased".
    //        If there was a default, the joint will be allowed to move without restriction.
    for(unsigned int i = 0; i < chain_main_.getNrOfJoints(); i++)
    {
        control_msgs::JointTolerance joint_tolerance;
        joint_tolerance.name = joint_names_[i];
        joint_tolerance.position = -1;
        joint_tolerance.velocity = -1;
        joint_tolerance.acceleration = -1;
        fjt_goal.path_tolerance.push_back(joint_tolerance);
        fjt_goal.goal_tolerance.push_back(joint_tolerance);
    }
    fjt_goal.goal_time_tolerance = ros::Duration(1.0);

    ROS_WARN_STREAM("FJT-Goal: " << fjt_goal);

    if ( goal->execute )
    {
        fjt_ac_->sendGoal(fjt_goal);

        while ( !lookat_as_->isPreemptRequested() )
        {
            bool finished_before_timeout = fjt_ac_->waitForResult(ros::Duration(0.1));

            actionlib::SimpleClientGoalState fjt_state = fjt_ac_->getState();
            message = "FJT State: " + fjt_state.toString();
            ROS_DEBUG_STREAM(lookat_name_ << ": " << message);

            /// fjt action successful?
            if (finished_before_timeout)
            {
                message = "FJT finished - State: " + fjt_state.toString() + ", ErrorCode: " + std::to_string(fjt_ac_->getResult()->error_code);

                if(fjt_state == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    success = true;
                    ROS_WARN_STREAM(lookat_name_ << ": " << message);
                    break;
                }
                else
                {
                    success = false;
                    ROS_ERROR_STREAM(lookat_name_ << ": " << message);
                    lookat_res_.success = success;
                    lookat_res_.message = message;
                    lookat_as_->setAborted(lookat_res_);
                    return;
                }
            }
        }
        if (lookat_as_->isPreemptRequested() )
        {
            fjt_ac_->cancelGoal();
            
            success = false;
            message = "Preempted during FJT execution";
            ROS_ERROR_STREAM(lookat_name_ << ": " << message);
            lookat_res_.success = success;
            lookat_res_.message = message;
            lookat_as_->setPreempted(lookat_res_);
            return;
        }
    }

    success = true;
    message = "Lookat finished successful.";
    ROS_WARN_STREAM(lookat_name_ << ": " << message);
    lookat_res_.success = success;
    lookat_res_.message = message;
    lookat_as_->setSucceeded(lookat_res_);
    return;
}
