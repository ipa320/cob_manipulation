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
 

#include <ros/ros.h>

#include <moveit/kinematics_base/kinematics_base.h>

#include <tf_conversions/tf_kdl.h>

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
#include "ikfast.h"

#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <urdf/model.h>

void print_frame(const char * str, const double* trans, const double* rot) {
    ROS_ERROR("%s %f %f %f, %f %f %f %f %f %f %f %f %f", str, trans[0], trans[1], trans[2], rot[0],  rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
}
void setConsistencyLimit(std::vector<std::pair<double, double> > &min_max,
        const std::vector<double> &seed, const std::vector<double> &consistency_limits) {
    if(min_max.size() != seed.size() || min_max.size() != consistency_limits.size()) return;

    for(unsigned int i=0; i <min_max.size(); ++i){
	min_max[i].first = fmax(min_max[i].first, seed[i] - consistency_limits[i]);
	min_max[i].second = fmin(min_max[i].second, seed[i] + consistency_limits[i]);
    }
}

struct Stepper {
    int upper_, lower_, current_;
    double *value;
    double start_, step_;
    Stepper(double *value, double lower, double upper, double step) :
        current_(0), value(value), start_(*value), step_(step) {
        double right_bound = fmax(lower, start_ - M_PI) + 2 * M_PI + step;
        double left_bound = fmin(upper, start_ + M_PI) - 2 * M_PI - step;
        right_bound = fmin(right_bound, upper);
        left_bound = fmax(left_bound, lower);

        lower_ = -(int) (fabs(left_bound - start_) / step);
        upper_ = fabs(right_bound - start_) / step;
        //ROS_ERROR("Stepper: %f %f %f %f, %d %d", *value, lower, upper, step, lower_, upper_);
    }
    void reset() {
        current_ = 0;
        *value = start_;
    }
    bool step() {
        int next;
        if (current_ <= 0) { // neg
            next = 1 - current_; // -> pos
            if (next > upper_) {
                next = -next; // -> neg
                if (next < lower_) return false;
            }
        } else { // pos
            next = -current_; // -> neg
            if (next < lower_) {
                next = 1 - next; // -> pos
                if (next > upper_) return false;
            }

        }
        current_ = next;
        *value = start_ + current_ * step_;
        return true;
    }
};

struct JointSpaceStepper {
    std::vector<Stepper> steppers;
    int current_;
    std::vector<double> state;
    JointSpaceStepper(double step, const std::vector<double> &ik_seed_state,
            const std::vector<std::pair<double, double> > &min_max,
            const std::vector<double> &indices) :
        current_(0), state(indices.size()) {
        int i = 0;
        for (std::vector<double>::const_reverse_iterator it = indices.rbegin(); it
                != indices.rend(); ++it) { // reverse!
            steppers.push_back(Stepper(&state[i++], min_max[*it].first,
                    min_max[*it].second, step));
        }
    }
    bool step() {
        bool overflow = false;
        while (current_ < steppers.size() && !steppers[current_].step()) {
            ++current_;
            overflow = true;
        }
        if (current_ >= steppers.size()) return false;
        if (overflow) {
            while (current_ > 0)
                steppers[current_--].reset();
        }
        return true;

    }
};

using namespace ikfast;

template<class A1, class A2>
std::ostream& operator<<(std::ostream& s, std::vector<A1, A2> const& vec)
{
    copy(vec.begin(), vec.end(), std::ostream_iterator<A1>(s, " "));
    return s;
}

class IkSolutionListFiltered: public IkSolutionList<double> {
protected:
    const std::vector<std::pair<double, double> > &min_max;
    const std::vector<double> &ik_seed_state;
    const kinematics::KinematicsBase::IKCallbackFn &solution_callback;
    const geometry_msgs::Pose &ik_pose;
    double min_dist;
    std::vector<double> min_solution;

    virtual bool filterSolution(const std::vector<double> &values) {
        for (unsigned int i = 0; i < min_max.size(); ++i)
            if (values[i] < min_max[i].first || values[i] > min_max[i].second) {
                return false;
            }

	moveit_msgs::MoveItErrorCodes error_code;
        error_code.val = error_code.SUCCESS;
        if (solution_callback) solution_callback(ik_pose, values, error_code);
        return error_code.val == error_code.SUCCESS;
    }
    double  ik_values[12];
public:
    IkSolutionListFiltered(
            const std::vector<std::pair<double, double> > &min_max,
            const std::vector<double> &ik_seed_state,
            const kinematics::KinematicsBase::IKCallbackFn  &solution_callback,
            const geometry_msgs::Pose &ik_pose) :
        min_max(min_max), ik_seed_state(ik_seed_state), solution_callback(
                solution_callback), ik_pose(ik_pose) {
        KDL::Frame frame;
        tf::poseMsgToKDL(ik_pose, frame);
        ik_values[0] = frame.p[0]; ik_values[1] = frame.p[1];  ik_values[2] = frame.p[2];
        ik_values[3] = frame.M.data[0]; ik_values[4] = frame.M.data[1]; ik_values[5] = frame.M.data[2];
        ik_values[6] = frame.M.data[3]; ik_values[7] = frame.M.data[4]; ik_values[8] = frame.M.data[5];
        ik_values[9] = frame.M.data[6]; ik_values[10] = frame.M.data[7]; ik_values[11] = frame.M.data[8];
    }

    virtual size_t AddSolution(const std::vector<
            IkSingleDOFSolutionBase<double> >& vinfos,
            const std::vector<int>& vfree) {

        //ROS_ERROR("Solution!");

        IkSolution<double> sol(vinfos, vfree);
        std::vector<double> vsolfree(sol.GetFree().size());
        std::vector<double> solvalues;
        sol.GetSolution(solvalues, vsolfree);


        double dist = harmonize(ik_seed_state, solvalues, min_max);
        if (filterSolution(solvalues)) {
            if (min_solution.empty() || dist < min_dist) min_solution
                    = solvalues;
        }

        return IkSolutionList<double>::AddSolution(vinfos, vfree);
    }
    static double harmonize(const std::vector<double> &ik_seed_state,
            std::vector<double> &solution, const std::vector<std::pair<double, double> > &min_max) {
        double dist_sqr = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (fabs(solution[i] - ik_seed_state[i]) > M_PI) {
                if (ik_seed_state[i] < solution[i]) {
                    if (solution[i] > 0 && solution[i] - 2 * M_PI >= min_max[i].first) solution[i] -= 2 * M_PI;
                } else {
                    if (solution[i] < 0  && solution[i] + 2 * M_PI <= min_max[i].second) solution[i] += 2 * M_PI;
                }
            }

            dist_sqr += fabs(solution[i] - ik_seed_state[i]);
        }
        return dist_sqr;
    }

    bool getMinSolution(std::vector<double> &dest) {
        if (min_solution.empty()) return false;
        dest = min_solution;
        return true;
    }
};
namespace IKFAST_NAMESPACE {

class IKFastPlugin: public kinematics::KinematicsBase {
public:
    /**
     * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
     * @param ik_link_name - the name of the link for which IK is being computed
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options) const {

        return searchPositionIK(ik_pose, ik_seed_state, solution, min_max_,
                error_code);
    }

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   double timeout,
                                   std::vector<double> &solution,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options) const {

        return searchPositionIK(ik_pose, ik_seed_state, solution, min_max_,
                error_code, timeout);
    }

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param the distance that the redundancy can be from the current position
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   double timeout,
                                   const std::vector<double> &consistency_limits,
                                   std::vector<double> &solution,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options) const {

        std::vector<std::pair<double, double> > min_max = min_max_;
        setConsistencyLimit(min_max, ik_seed_state, consistency_limits);
        return searchPositionIK(ik_pose, ik_seed_state, solution, min_max,
                error_code, timeout);
    }

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   double timeout,
                                   std::vector<double> &solution,
                                   const IKCallbackFn &solution_callback,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options) const {

        return searchPositionIK(ik_pose, ik_seed_state, solution, min_max_,
                error_code, timeout, solution_callback);
    }

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
     * around those specified in the seed state are admissible and need to be searched.
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param consistency_limit the distance that the redundancy can be from the current position
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   double timeout,
                                   const std::vector<double> &consistency_limits,
                                   std::vector<double> &solution,
                                   const IKCallbackFn &solution_callback,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options) const {

        std::vector<std::pair<double, double> > min_max = min_max_;
        setConsistencyLimit(min_max, ik_seed_state, consistency_limits);
        return searchPositionIK(ik_pose, ik_seed_state, solution, min_max,
                error_code, timeout, solution_callback);
    }

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
     * @param response - the response contains stamped pose information for all the requested links
     * @return True if a valid solution was found, false otherwise
     */
	virtual bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::Pose> &poses) const {
        KDL::Frame p_out;

        if (joint_angles.size() != GetNumJoints()) {
            ROS_ERROR("%d joint angles are required", GetNumJoints());
            return false;
        }

        if (link_names.size() != 1 || link_names[0] != tip_frame_) {
            ROS_ERROR("Can compute FK for %s only",tip_frame_.c_str());
            return false;
        }

        ComputeFk(&joint_angles[0], p_out.p.data, p_out.M.data);
        // print_frame("FK:", p_out.p.data, p_out.M.data);
        poses.resize(1);
        tf::poseKDLToMsg(p_out, poses[0]);
        return true;
    }

    /**
     * @brief  Initialization function for the kinematics
     * @return True if initialization was successful, false otherwise
     */
    virtual bool initialize(const std::string& robot_description,
                          const std::string& group_name,
                          const std::string& base_frame,
                          const std::string& tip_frame,
                          double search_discretization){
        setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

        links_.resize(1);
        links_[0] = tip_frame_;

        // Load and Read Models
        if (!loadModel(robot_description)) {
            ROS_FATAL("Could not load models!");
            return false;
        }
        indices_.clear();
        for (int i = 0; i < GetNumFreeParameters(); ++i)
            indices_.push_back(GetFreeParameters()[i]);
	return true;
    }

    /**
     * @brief  Return all the joint names in the order they are used internally
     */
    virtual const std::vector<std::string>& getJointNames() const {
        return joints_;
    }

    /**
     * @brief  Return all the link names in the order they are represented internally
     */
    virtual const std::vector<std::string>& getLinkNames() const {
        return links_;
    }

    /**
     * @brief  Virtual destructor for the interface
     */
    virtual ~IKFastPlugin() {
    }
    IKFastPlugin() :
        bounds_epsilon_(0.0) {
    }
protected:
    std::vector<std::pair<double, double> > min_max_;
    double bounds_epsilon_;
    std::vector<double> indices_;
    std::vector<std::string> links_;
    std::vector<std::string> joints_;
    bool searchPositionIK(
            const geometry_msgs::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            std::vector<double> &solution,
            const std::vector<std::pair<double, double> > &min_max,
            moveit_msgs::MoveItErrorCodes &error_code,
            const double timeout = -1,
            const IKCallbackFn &solution_callback = 0) const {

        KDL::Frame frame;
        tf::poseMsgToKDL(ik_pose, frame);
        error_code.val = error_code.NO_IK_SOLUTION;

		if(ik_seed_state.size() < GetNumJoints()){
            ROS_ERROR_STREAM("Needs " << GetNumJoints() << "joint values. but only " << ik_seed_state.size() << "were given.");
	    	return false;
		}

        // print_frame("IK:", frame.p.data, frame.M.data);
        JointSpaceStepper jss(search_discretization_, ik_seed_state, min_max,
                indices_);

        IkSolutionListFiltered sollist(min_max, ik_seed_state,
                solution_callback, ik_pose);

        ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
        do {
            //ROS_ERROR("State: %f", jss.state[0]);
            if (ComputeIk(frame.p.data, frame.M.data, indices_.empty()?0:&jss.state[0], sollist)) {
                error_code.val = error_code.START_STATE_IN_COLLISION; // is reachable but violates constraints
            }
            if (sollist.getMinSolution(solution)) {
                error_code.val = error_code.SUCCESS;
                break;
            }
            if (timeout >= 0.0 && ros::Time::now() > maxTime) {
                error_code.val = error_code.TIMED_OUT;
                break;
            }
        } while (jss.step());
        return error_code.val == error_code.SUCCESS;
    }
    bool loadModel(const std::string param) {
        urdf::Model robot_model;

        if (!robot_model.initParam(param)) {
            ROS_FATAL("Could not initialize robot model");
            return false;
        }
        if (!readJoints(robot_model)) {
            ROS_FATAL("Could not read information about the joints");
            return false;
        }
        return true;
    }

    bool readJoints(urdf::Model &robot_model) {
        int dimension_ = 0;
        // get joint maxs and mins
        boost::shared_ptr<const urdf::Link> link = robot_model.getLink(
                tip_frame_);
        boost::shared_ptr<const urdf::Joint> joint;
        while (link && link->name != base_frame_) {
            joint = robot_model.getJoint(link->parent_joint->name);
            if (!joint) {
                ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
                return false;
            }
            if (joint->type != urdf::Joint::UNKNOWN && joint->type
                    != urdf::Joint::FIXED) {
                ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
                dimension_++;
            }
            link = robot_model.getLink(link->getParent()->name);
        }

        if (dimension_ != GetNumJoints()) {
            ROS_ERROR("Solver needs %d joints, but %d were found.",GetNumJoints(), dimension_);
            return false;
        }

        min_max_.resize(dimension_);
        joints_.resize(dimension_);
        link = robot_model.getLink(tip_frame_);

        unsigned int i = 0;
        while (link && i < dimension_) {
            joint = robot_model.getJoint(link->parent_joint->name);
            if (joint->type != urdf::Joint::UNKNOWN && joint->type
                    != urdf::Joint::FIXED) {
                ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

                float lower, upper;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit
                                + bounds_epsilon_;
                        upper = joint->safety->soft_upper_limit
                                - bounds_epsilon_;
                    } else {
                        lower = joint->limits->lower + bounds_epsilon_;
                        upper = joint->limits->upper - bounds_epsilon_;
                    }
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                }
                min_max_[dimension_ - i - 1] = std::make_pair(lower, upper);
                joints_[dimension_ - i - 1] = joint->name;
                i++;
            }
            link = robot_model.getLink(link->getParent()->name);
        }

        return true;
    }
};

}
#include <pluginlib/class_list_macros.h>

#define ADD_TYPED_PLUGIN(type,name) PLUGINLIB_DECLARE_CLASS(cob_kinematics, type##name, IKFAST_NAMESPACE::IKFastPlugin, kinematics::KinematicsBase)
#define ADD_IKFAST_PLUGIN(name) ADD_TYPED_PLUGIN(IKFast_, name)
ADD_IKFAST_PLUGIN(IKFAST_NAMESPACE)
;

