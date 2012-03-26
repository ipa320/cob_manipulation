#include <cob_mmcontroller/cob_cartesian_trajectories_PID.h>

cob_cartesian_trajectories::cob_cartesian_trajectories() : as_model_(n, "moveModel", boost::bind(&cob_cartesian_trajectories::moveModelActionCB, this, _1), false)
{
    ros::NodeHandle node;
    node.param("cob_cartesian_trajectories_PID/p_gain", p_gain_, 1.0);
    node.param("cob_cartesian_trajectories_PID/i_gain", i_gain_, 0.002);
    node.param("cob_cartesian_trajectories_PID/d_gain", d_gain_, 0.0);
    ROS_INFO("Starting PID controller with P: %e, I: %e, D: %e", p_gain_, i_gain_, d_gain_);
    
    cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
    joint_state_sub_ = n.subscribe("/joint_states", 1, &cob_cartesian_trajectories::jointStateCallback, this);
    cart_command_pub = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
    debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/mm/debug",1);
    serv_prismatic = n.advertiseService("/mm/move_pri", &cob_cartesian_trajectories::movePriCB, this);      // new service
    serv_rotational = n.advertiseService("/mm/move_rot", &cob_cartesian_trajectories::moveRotCB, this);     // new service
    serv_model = n.advertiseService("/mm/move_model", &cob_cartesian_trajectories::moveModelCB, this);       // new service to work with models
    map_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    twist_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);   // publish twist to be visualized inj rviz
    track_pub_ = n.advertise<articulation_msgs::TrackMsg>("/track", 1);                 // publish generated trajectory for debugging
    model_pub_ = n.advertise<articulation_msgs::ModelMsg>("/model", 1);                 // publish given model for debugging
    bRun = false;
    as_model_.start();
    targetDuration = 0;
    currentDuration = 0;
    mode = "prismatic";     // prismatic, rotational, trajectory, model
   
    pub_timer = ros::Time::now();
    pub_counter = 0;
    double const PI = 4.0*std::atan(1.0);

    getJointLimits(UpperLimits, LowerLimits);   // get arm joint limits from topic /robot_description

    tf::TransformBroadcaster br;

    //int axis_center;    // axis of F_handle pointing to the rotational axis of articulation
    
    bHandle = false;
    debug = true;
}


// to avoid reaching the arm joint limits
// get arm joint limits from topic /robot_description
void cob_cartesian_trajectories::getJointLimits(std::vector<double> &UpperLimits, std::vector<double> &LowerLimits)
{
    ros::NodeHandle param_node;
    const unsigned int DOF = 7;
    std::vector<std::string> JointNames;
    std::string param_name = "robot_description";
    std::string full_param_name;
    std::string xml_string;

    for (unsigned int i = 1; i <= DOF; i++)
    {
        stringstream ss;
        ss << i;
        JointNames.push_back("arm_" + ss.str() + "_joint");
    }

    param_node.searchParam(param_name, full_param_name);
    if (param_node.hasParam(full_param_name))
    {
        param_node.getParam(full_param_name.c_str(), xml_string);
        //std::cout << "Parameter name: " << full_param_name << "\n";
    }

    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
        param_node.shutdown();
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
        param_node.shutdown();
    }
    ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        param_node.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");

    /// Get lower limits out of urdf model
    for (unsigned int i = 0; i < DOF; i++)
    {
        LowerLimits.push_back(model.getJoint(JointNames[i].c_str())->limits->lower);
    }

    // Get upper limits out of urdf model
    for (unsigned int i = 0; i < DOF; i++)
    {
        UpperLimits.push_back(model.getJoint(JointNames[i].c_str())->limits->upper);
    }
    param_node.shutdown();
}

// get arm joint states 
std::vector<double> cob_cartesian_trajectories::parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
    std::vector<double> q_temp(7);
    bool parsed = false;
    unsigned int count = 0;
    for(unsigned int i = 0; i < names.size(); i++)
    {
            if(strncmp(names[i].c_str(), "arm_", 4) == 0)
            {
                q_temp[count] = positions[i];
                count++;
                parsed = true;
            }
    }

    if(!parsed)
        return q_last;

    q_last = q_temp;
    return q_temp;

}

// check if an arm joint limit is reached
void cob_cartesian_trajectories::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (bRun)
    {
        std::vector<std::string> names = msg->name;
        std::vector<double> positions = msg->position;
        jointStates = parseJointStates(names,positions);

        // stopping to run trajectory 
        for (unsigned int i = 0; i < jointStates.size(); i++)
        {
            if (jointStates[i] <= (LowerLimits[i] + 0.04))
            {
                ROS_INFO("Stopping trajectory because arm joint %d reached almost lower joint limit!", i+1);
                result_.exit_code = 2;
                stopTrajectory();
                //std::cout << "Stopping trajectory because arm joint " << i+1 << " reached almost lower joint limit!" << "\n";
            }
            else if (jointStates[i] >= (UpperLimits[i] - 0.04))
            {
                ROS_INFO("Stopping trajectory because arm joint %d reached almost upper joint limit!", i+1);
                result_.exit_code = 2;
                stopTrajectory();
                //std::cout << "Stopping trajectory because arm joint " << i+1 << " reached almost upper joint limit!" << "\n";
            }
            //std::cout << "arm_" << i+1 << "_joint is " << jointStates[i] << " >> " << -1.0 *(fabs(LowerLimits[i])+jointStates[i]) << " to lower and " << UpperLimits[i]-jointStates[i] << " to upper limit" << "\n";
        }
    }

}


// action for model 
void cob_cartesian_trajectories::moveModelActionCB(const cob_mmcontroller::ArticulationModelGoalConstPtr& goal)
{
    articulation_msgs::ModelMsg pub_model;
    //tf broadcaster 
    tf::Transform transform_articulation;

    mode = goal->model.name;
    std::cout << "Mode:" << mode << "\n";
    targetDuration = goal->target_duration.toSec();
    params = goal->model.params;

    //set up articulation frame
    if (mode == "rotational")
    {
        transform_articulation.setOrigin( tf::Vector3(getParamValue("rot_center.x"), getParamValue("rot_center.y"), getParamValue("rot_center.z")) );
        transform_articulation.setRotation( tf::Quaternion(getParamValue("rot_axis.x"), getParamValue("rot_axis.y"), getParamValue("rot_axis.z"), getParamValue("rot_axis.w")) );
    }
    else
    {
        transform_articulation.setOrigin( tf::Vector3(getParamValue("rigid_position.x"), getParamValue("rigid_position.y"), getParamValue("rigid_position.z")) );
        transform_articulation.setRotation( tf::Quaternion(getParamValue("rigid_orientation.x"), getParamValue("rigid_orientation.y"), getParamValue("rigid_orientation.z"), getParamValue("rigid_orientation.w")) );
    }
    //publish articulation frame
    br.sendTransform(tf::StampedTransform(transform_articulation, ros::Time::now(), "/map", "/articulation_center"));

    if(start())
    {
        while(bRun)
        {
            //wait until finished
            //publish articulation frame TODO: necessary
            br.sendTransform(tf::StampedTransform(transform_articulation, ros::Time::now(), "/map", "/articulation_center"));
        
            //publish model
            pub_model = goal->model;
            pub_model.header.stamp = ros::Time::now();
            pub_model.header.frame_id = "/map";

            model_pub_.publish(pub_model);
            //publish feedback
            feedback_.time_left = targetDuration - currentDuration;
            as_model_.publishFeedback(feedback_);

            sleep(1);
        }
        if (result_.exit_code != 0)
            as_model_.setAborted(result_);
        else
            as_model_.setSucceeded(result_);
    }
    return;
}

bool cob_cartesian_trajectories::movePriCB(cob_mmcontroller::MovePrismatic::Request& request, cob_mmcontroller::MovePrismatic::Response& response)    //TODO // prismatic callback
{
    mode = "prismatic";
    targetDuration = request.target_duration.toSec();
    params = request.params;
    std::cout << targetDuration << "\n";
    return start();
}

bool cob_cartesian_trajectories::moveRotCB(cob_mmcontroller::MoveRotational::Request& request, cob_mmcontroller::MoveRotational::Response& response)    //TODO // rotational callback
{
    mode = "rotational";
    targetDuration = request.target_duration.toSec();
    params = request.params;
    std::cout << targetDuration << "\n";
    return start();
}

/*bool cob_cartesian_trajectories::moveTrajCB(cob_mmcontroller::MoveTrajectory::Request& request, cob_mmcontroller::MoveTrajectory::Response& response)   // TODO // trajectory callback
{
    mode = "trajectory";
    targetDuration = request.target_duration.toSec();
    track = request.pose
    return start();
}*/

bool cob_cartesian_trajectories::moveModelCB(cob_mmcontroller::MoveModel::Request& request, cob_mmcontroller::MoveModel::Response& response)  //TODO // model callback
{
    mode = "model";
    targetDuration = request.target_duration.toSec();
    //model_params = request.model.params            // model parameters
    track = request.model.track.pose_projected;         // trajectory
    return start();
}

bool cob_cartesian_trajectories::start() //TODO request->model.params // start 
{
    if(bRun)
    {
        ROS_ERROR("Already running trajectory");
        return false;
    }
    else
    {
        bRun = true;
        bStarted = false;
        timer = ros::Time::now();
        tstart = ros::Time::now();
        currentDuration = 0;
        trajectory_points.clear();
        result_.exit_code = 1;
        Error_last = Twist::Zero();
        return true;
    }    
}

//Pose is global pose with odometry
void cob_cartesian_trajectories::cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(bRun)
    {
        ros::Duration dt = ros::Time::now() - timer;
        cout << "\n" << "===================================" << "\n\n" << "dt: " << dt << "\n";
        timer = ros::Time::now();
        if((targetDuration-currentDuration) <= 0)
        {
            geometry_msgs::Twist twist;
            cart_command_pub.publish(twist);
            ROS_INFO("finished trajectory in %f", ros::Time::now().toSec() - tstart.toSec());
            result_.exit_code = 0;
            stopTrajectory();
            return;
        }
        KDL::Frame current;
        KDL::Frame myhinge;
        tf::PoseMsgToKDL(msg->pose, current);
        tf::PoseMsgToKDL(current_hinge.pose, myhinge);
        KDL::Vector unitz = myhinge.M.UnitZ();
        std::cout << "Radius because of Hinge: " << (myhinge.p - current.p) << "UnitZ of hinge: " << unitz.z() << "\n";

        // get twist to be published
        geometry_msgs::Twist twist;
        twist = getTwist(currentDuration, current); 

        // publish twist
        cart_command_pub.publish(twist);

        // add needed time
        currentDuration+=dt.toSec();

        // add position to be visualized in rviz
        geometry_msgs::Point p;
        p.x = msg->pose.position.x;
        p.y = msg->pose.position.y;
        p.z = msg->pose.position.z;
        trajectory_points.push_back(p);
    }
    else
    {
        //publish zero    
        geometry_msgs::Twist twist;
        cart_command_pub.publish(twist);
    }
}

void cob_cartesian_trajectories::stopTrajectory()
{
    geometry_msgs::Twist twist;
    cart_command_pub.publish(twist);
    bRun = false;
    bStarted = false;
    sendMarkers();
    Error = Twist::Zero();
    Error_sum = Twist::Zero();
    Error_dot = Twist::Zero();
}

geometry_msgs::Twist cob_cartesian_trajectories::getTwist(double dt, Frame F_current)
{
    KDL::Frame F_target;
    KDL::Frame F_diff;
    geometry_msgs::Twist ControllTwist;
    double start_roll, start_pitch, start_yaw = 0.0;
    double current_roll, current_pitch, current_yaw = 0.0;
    double target_roll, target_pitch, target_yaw = 0.0;

    if(!bStarted)
    {
        F_EE_start = F_current;
        F_EE_start.M.GetRPY(last_rpy_angles["target_roll"], last_rpy_angles["target_pitch"], last_rpy_angles["target_yaw"]);
        F_EE_start.M.GetRPY(last_rpy_angles["current_roll"], last_rpy_angles["current_pitch"], last_rpy_angles["current_yaw"]);
        bStarted = true;
        bHandle = true;
    }
    
    getTargetPosition(dt, F_target);
   
    F_EE_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);

    //std::cout << "AngleDiff: " << (current_yaw-start_yaw) << " vs " << (soll_angle) << " error: " << (soll_angle-current_yaw-start_yaw) << "\n";

    
    std::cout << "Target (x,y):  " << F_target.p.x() << ", " << F_target.p.y() << "\n";
    std::cout << "Error (x,y):   " << F_target.p.x()-F_current.p.x() << ", " << F_target.p.y()-F_current.p.y() << "\n";
    std::cout << "Start (x,y):   " << F_EE_start.p.x() << ", " << F_EE_start.p.y() << "\n";
    std::cout << "Current (x,y): " << F_current.p.x() << ", " << F_current.p.y() << "\n";

    // calling PIDController to calculate the actuating variable and get back twist
    ControllTwist = PIDController(dt, F_target, F_current);
    
    //DEBUG
    F_diff.p.x(F_target.p.x()-F_current.p.x());
    F_diff.p.y(F_target.p.y()-F_current.p.y());
    F_diff.p.z(F_target.p.z()-F_current.p.z());
    geometry_msgs::PoseArray poses;
    poses.poses.resize(3);
    tf::PoseKDLToMsg(F_current, poses.poses[0]);
    tf::PoseKDLToMsg(F_target, poses.poses[1]);
    tf::PoseKDLToMsg(F_diff, poses.poses[2]);
    debug_cart_pub_.publish(poses);
    //

    return ControllTwist;
}

void cob_cartesian_trajectories::getTargetPosition(double dt, KDL::Frame &F_target)    //TODO //
{
    if (mode == "prismatic")
        getPriTarget(dt, F_target);
    else if (mode == "rotational")
        getRotTarget(dt, F_target);
    /*else if (mode == "trajectory");
        getTrajTarget(dt, F_target);
    else if (mode == "model");
        getModelTarget(dt, F_target);*/
    else
    {
        ROS_ERROR("Invalid mode");
        F_target = F_EE_start;
    }
}

// linear trajectory 
void cob_cartesian_trajectories::getPriTarget(double dt, KDL::Frame &F_target)
{
    double length;
    double partial_length;
    double pris_dir_x;
    double pris_dir_y;
    double pris_dir_z;
    
    length = getParamValue("length");
    pris_dir_x = getParamValue("prismatic_dir.x");
    pris_dir_y = getParamValue("prismatic_dir.y");
    pris_dir_z = getParamValue("prismatic_dir.z");
    
    partial_length = length * (dt/targetDuration);
    
    F_target.p.x(F_EE_start.p.x() + partial_length*pris_dir_x);
    F_target.p.y(F_EE_start.p.y() + partial_length*pris_dir_y);
    F_target.p.z(F_EE_start.p.z());
    //F_target.p.z(F_EE_start.p.z() + partial_length*pris_dir_z);
    F_target.M = F_EE_start.M; 
    
    std::cout << "F_X: " << F_target.p.x() << " F_Y: " << F_target.p.y() << " F_Z: " << F_target.p.z() << "\n";
}


//rotational 6D-trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;

    KDL::Frame F_track;

    // get start frame of trajectory
    if (bHandle)
        getRotStart(F_track_start);

    angle = getParamValue("action");

    // calculating partial_angle
    partial_angle = angle * (dt/targetDuration);

    // creating trajectory frame w.r.t. the track_start frame
    // orientation is like sdh_tip_link frame
    F_track.p[axis_center] = rot_radius*(1-cos(partial_angle)) * opening_side;
    F_track.p[2] = -rot_radius*sin(partial_angle) * opening_side;

    if (axis_center == 0)
        F_track.M.DoRotY(-partial_angle);
    else if (axis_center == 1)
        F_track.M.DoRotX(partial_angle);
    else
        ROS_ERROR("Wrong rotation axis in F_track");

    // transformation of trajectory frame into base_link frame ??? map frame
    F_target.p = F_track_start*F_track.p;       // transform F_Track in F_track_start 
    F_target.M = F_EE_start.M*F_track.M;        // rotation with respect to gripper start frame (F_EE_start)

    // tf transform F_track_start
    tf::Transform transform_track_start;
    tf::TransformKDLToTF(F_track_start, transform_track_start);
    br.sendTransform(tf::StampedTransform(transform_track_start, ros::Time::now(), "/map", "/track_start"));
    
    // tf transform F_track
    tf::Transform transform_track;
    tf::TransformKDLToTF(F_track, transform_track);
    br.sendTransform(tf::StampedTransform(transform_track, ros::Time::now(), "/track_start", "/track"));

    // tf transform F_target
    tf::Transform transform_target;
    tf::TransformKDLToTF(F_target, transform_target);
    br.sendTransform(tf::StampedTransform(transform_target, ros::Time::now(), "/map", "/target"));
}


//calculate start position of trajectory from rot_axis, rot_radius and gripper position
void cob_cartesian_trajectories::getRotStart(KDL::Frame &F_handle)
{
    // auxiliary variables
    double rot_radius_actual;

    int axis_no;

    KDL::Frame F_articulation;
    
    Eigen::Vector3d articulation_Z;
    Eigen::Vector3d articulation_O;
    Eigen::Vector3d handle_O;
    Eigen::Vector3d perpendicular;
    Eigen::Vector3d perpendicular_EE;
    Eigen::Vector3d trans_ee_art_EE;

    Eigen::Hyperplane<double, 3> handle_plane;
    
    map<int, KDL::Vector> handle_rot;

    //tf transform
    tf::Transform transform_handle;
    
    // set up articulation frame
    F_articulation.p.x(getParamValue("rot_center.x"));
    F_articulation.p.y(getParamValue("rot_center.y"));
    F_articulation.p.z(getParamValue("rot_center.z"));
    F_articulation.M = KDL::Rotation::Quaternion(getParamValue("rot_axis.x"), getParamValue("rot_axis.y"), getParamValue("rot_axis.z"), getParamValue("rot_axis.w"));
    debug ? (std::cout << "F_articulation" <<  F_articulation << "\n" ) : (std::cout << ""); //debug

    // EE start position is sdh_tip_link frame
    debug ? (std::cout << "F_EE_start" <<  F_EE_start << "\n") : (std::cout << ""); //debug

    // origin of track start frame correlates with EE start
    F_handle.p = F_EE_start.p;

    // z-axis of articulation frame in global coordinates to use as normal for rotation plane
    KDL::Vector articulation_Z_KDL = F_articulation.M.UnitZ(); 
    vector3dKDLToEigen(articulation_Z_KDL, articulation_Z);
    std::cout << "articulation_Z_KDL" << "\n" <<  articulation_Z_KDL << "\n"; //debug
    std::cout << "articulation_Z" << "\n" <<  articulation_Z << "\n"; //debug

    // origin of handle frame as point in plane of rotation
    vector3dKDLToEigen(F_handle.p, handle_O);
    std::cout << "handle_O" << "\n" <<  handle_O << "\n"; //debug

    // origin of articulation frame
    vector3dKDLToEigen(F_articulation.p, articulation_O);
    std::cout << "articulation_O" << "\n" <<  articulation_O << "\n"; //debug

    // handle plane calculation --> three point 
    handle_plane = Eigen::Hyperplane<double, 3>::Through(articulation_O, articulation_O + articulation_Z, handle_O);
    std::cout << "handle_plane_offset" << "\n" <<  handle_plane.offset() << "\n"; //debug
    std::cout << "handle_plane_coeffs" << "\n" <<  handle_plane.coeffs() << "\n"; //debug
    std::cout << "handle_plane_distance to articulation_O" << "\n" <<  handle_plane.absDistance(articulation_O) << "\n"; //debug
    std::cout << "handle_plane_distance to handle" << "\n" <<  handle_plane.absDistance(handle_O) << "\n"; //debug

    // perpendicular of articulation_Z through handle_O (already normalized)
    perpendicular = articulation_Z.cross(handle_plane.normal());
    std::cout << "perdendicular" << "\n" <<  perpendicular << "\n"; //debug
    if (perpendicular.norm() < 1e-6)
        ROS_ERROR("Normals are parallel");

    std::cout << "perdendicular norm" << "\n" <<  perpendicular.norm() << "\n"; //debug
    std::cout << "perpendicular normalized" << "\n" <<  perpendicular/perpendicular.norm() << "\n"; //debug

    // translation from EE start to articulation in global coord
    KDL::Vector trans_ee_art_KDL = F_articulation.p - F_EE_start.p;
    std::cout << "trans_ee_art_KDL" << "\n" <<  trans_ee_art_KDL << "\n"; //debug

    // transform trans_ee_art into F_EE_start
    KDL::Vector trans_ee_art_KDL_EE = F_EE_start.M.Inverse()*trans_ee_art_KDL;
    std::cout << "trans_ee_art_KDL_EE" << "\n" <<  trans_ee_art_KDL_EE << "\n"; //debug

    // transform perpendicular into EE frame
    KDL::Vector perpendicular_EE_KDL = F_EE_start.M.Inverse()*KDL::Vector(perpendicular[0], perpendicular[1], perpendicular[2]);
    std::cout << "perpendicular_EE_KDL" << "\n" <<  perpendicular_EE_KDL << "\n"; //debug
    vector3dKDLToEigen(perpendicular_EE_KDL, perpendicular_EE);
    std::cout << "perpendicular_EE" << "\n" <<  perpendicular_EE << "\n"; //debug

    // get actual rot_radius
    rot_radius_actual = abs(dot(trans_ee_art_KDL_EE, perpendicular_EE_KDL));
    std::cout << "rot_radius_actual" << "\n" <<  rot_radius_actual << "\n"; //debug

    // normalize trans_ee_art_KDL(_EE)
    trans_ee_art_KDL.Normalize();
    trans_ee_art_KDL_EE.Normalize();

    // get axis pointing on articulation --> use maxCoeff of absolute x and y axes 
    vector3dKDLToEigen(trans_ee_art_KDL_EE, trans_ee_art_EE);
    // take only x and y axes 
    Eigen::Vector2d xORy = Eigen::Vector2d(abs(trans_ee_art_EE[0]), abs(trans_ee_art_EE[1]));
    xORy.maxCoeff(&axis_no);  // index correlates with axis
    std::cout << "axis_no" << "\n" << axis_no << "\n"; //debug

    // check if articulation origin is on the right (positive) or left (negative) side w.r.t. the handle frame
    // meaning the current door is opening on the left or the right side
    // and check and set direction of perpendicular depending on this
    if (trans_ee_art_EE[axis_no] > 0.0)
    {
        opening_side = 1.0;
        if (sign(trans_ee_art_EE[axis_no]) != sign(perpendicular_EE[axis_no]))
            perpendicular *= (-1.0);
            perpendicular_EE *= (-1.0);
    }
    else
    {
        opening_side = -1.0;
        if (sign(trans_ee_art_EE[axis_no]) == sign(perpendicular_EE[axis_no]))
            perpendicular *= (-1.0);
            perpendicular_EE *= (-1.0);
    }
    std::cout << "opening side" << "\n" <<  opening_side << "\n"; //debug
    std::cout << "perpendicular" << "\n" <<  perpendicular << "\n"; //debug
    std::cout << "perpendicular_EE" << "\n" <<  perpendicular_EE << "\n"; //debug

    // collect vectors for F_handle orientation
    vector3dEigenToKDL(perpendicular, handle_rot[axis_no]);

    // check direction of articulation_Z and set up second vector of handle_rot (parallel to articulation_Z)
    std::cout << "other axis" << "\n" << abs(axis_no-1) << "\n"; //debug
    KDL::Vector articulation_Z_KDL_EE = F_EE_start*articulation_Z_KDL;
    if (articulation_Z_KDL_EE.z() < 0.0)
        handle_rot[abs(axis_no-1)] = (-1.0)*articulation_Z_KDL;
    else
        handle_rot[abs(axis_no-1)] = articulation_Z_KDL;
    // calculate cross produkt to get handle_rot z-axis 
    if (axis_no == 0)
    {
        Eigen::Vector3d handle_Z = perpendicular.cross(articulation_Z);
        handle_rot[2] = KDL::Vector(handle_Z[0], handle_Z[1], handle_Z[2]);
    }   
    else if (axis_no == 1)
    {
        Eigen::Vector3d handle_Z = articulation_Z.cross(perpendicular);
        handle_rot[2] = KDL::Vector(handle_Z[0], handle_Z[1], handle_Z[2]);
    }
    else
        ROS_ERROR("Wrong axis");
    std::cout << "rot vector x" << "\n" << handle_rot[0] << "\n"; //debug
    std::cout << "rot vector y" << "\n" << handle_rot[1] << "\n"; //debug
    std::cout << "rot vector z" << "\n" << handle_rot[2] << "\n"; //debug

    // set up F_handle rotation
    F_handle.M = KDL::Rotation(handle_rot[0], handle_rot[1], handle_rot[2]);
    std::cout << "F_handle" << "\n" << F_handle << "\n"; //debug

    // broadcast F_handle
    tf::TransformKDLToTF(F_handle, transform_handle);
    br.sendTransform(tf::StampedTransform(transform_handle, ros::Time::now(), "/map", "/handle"));
    
    // calculate, check and set up rot_radius TODO: abort if tolerance is exceeded
    rot_radius = getParamValue("rot_radius");
    if (rot_radius == 0.0)
    {
        ROS_DEBUG("Parameter rot_radius no set, rot_radius_actual will be taken!");
    }
    std::cout << "radius difference" << "\n" << rot_radius-rot_radius_actual << "\n"; //debug
    if (abs(rot_radius - rot_radius_actual) > 0.05)
    {
        std::cout << "rot_radius" << "\n" << rot_radius << "\n"; //debug
        std::cout << "rot_radius_actual" << "\n" << rot_radius_actual << "\n"; //debug
        ROS_ERROR("model radius and actual radius differ quite much");
    }
    rot_radius = rot_radius_actual;

    axis_center = axis_no;
    bHandle = false;
}


double cob_cartesian_trajectories::getParamValue(std::string param_name)
{
    for (unsigned int i = 0; i < params.size(); i++)
    {
        if (strncmp(params[i].name.c_str(), param_name.c_str(), param_name.size()) == 0)
        {
            std::cout << " -- " << params[i].name << " -- " << params[i].value << "\n";
            return params[i].value;
        }
    }
    ROS_ERROR("Missing parameter");
    std::cout << "No value found for parameter " << param_name << "\n";
    return 0.0;
}


geometry_msgs::Twist cob_cartesian_trajectories::PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_current)
{
    geometry_msgs::Twist twist;
    double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;
    double target_roll = 0.0, target_pitch = 0.0, target_yaw = 0.0;
    
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    target_yaw = unwrapRPY("target_yaw", target_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
    current_yaw = unwrapRPY("current_yaw", current_yaw);
        
    Error.vel.x(F_target.p.x() - F_current.p.x());
    Error.vel.y(F_target.p.y() - F_current.p.y());
    Error.vel.z(F_target.p.z() - F_current.p.z());
    Error.rot.x(target_roll - current_roll);
    Error.rot.y(target_pitch - current_pitch);
    Error.rot.z(target_yaw - current_yaw);
    
    cout << "Error twist: " << "\n" << Error << "\n";
    
    Error_sum.vel.x(Error_sum.vel.x() + Error.vel.x() * dt);
    Error_sum.vel.y(Error_sum.vel.y() + Error.vel.y() * dt);
    Error_sum.vel.z(Error_sum.vel.z() + Error.vel.z() * dt);
    Error_sum.rot.x(Error_sum.rot.x() + Error.rot.x() * dt);
    Error_sum.rot.y(Error_sum.rot.y() + Error.rot.y() * dt);
    Error_sum.rot.z(Error_sum.rot.z() + Error.rot.z() * dt); 
    
    cout << "Error_sum twist: " << "\n" << Error_sum << "\n";
    
    Error_dot.vel.x((Error.vel.x() - Error_last.vel.x()) / dt);
    Error_dot.vel.y((Error.vel.y() - Error_last.vel.y()) / dt);
    Error_dot.vel.z((Error.vel.z() - Error_last.vel.z()) / dt);
    Error_dot.rot.x((Error.rot.x() - Error_last.rot.x()) / dt);
    Error_dot.rot.y((Error.rot.y() - Error_last.rot.y()) / dt);
    Error_dot.rot.z((Error.rot.z() - Error_last.rot.z()) / dt);
    
    cout << "Error_dot twist: " << "\n" << Error_dot << "\n";
    
    // create twist
    twist.linear.x = p_gain_*Error.vel.x() + i_gain_*Error_sum.vel.x();
    twist.linear.y = p_gain_*Error.vel.y() + i_gain_*Error_sum.vel.y(); 
    twist.linear.z = p_gain_*Error.vel.z();//p_gain_*Error.vel.z());
    twist.angular.x = 0.0;//p_gain_*Error.rot.x());//(p_gain_*Error.rot.x() + i_gain_*Error_sum.rot.x());
    twist.angular.y = 0.0;//p_gain_*Error.rot.y());//(p_gain_*Error.rot.y() + i_gain_*Error_sum.rot.y());
    twist.angular.z = p_gain_*Error.rot.z() + i_gain_*Error_sum.rot.z();

    pubTrack(1, ros::Duration(0.5), F_target);
    pubTrack(9, ros::Duration(0.5), F_current);
    pubTwistMarkers(ros::Duration(1.0), twist, F_current);
    
    Error_last.vel.x(Error.vel.x());
    Error_last.vel.y(Error.vel.y());
    Error_last.vel.z(Error.vel.z());
    Error_last.rot.x(Error.rot.x());
    Error_last.rot.y(Error.rot.y());
    Error_last.rot.z(Error.rot.z());

    return twist;
}


//VISUALIZATION IN RVIZ
//
// publish generated trajectory
void cob_cartesian_trajectories::pubTrack(const int track_id, const ros::Duration pub_duration, const KDL::Frame &F_pub)
{
    articulation_msgs::TrackMsg track;
    // set up a new TrackMsg
    if (track_map.find(track_id) == track_map.end())
    {
        cout << "new track: " << track_id << "\n";
        track.header.stamp = (ros::Time::now() - pub_duration);
        track.header.frame_id = "/map";
        track.id = track_id;
        track_map[track_id] = track;
    }

    // store pose in corresponding TrackMsg
    if ((ros::Time::now() - track_map[track_id].header.stamp) >= pub_duration)
    {
        track_map[track_id].header.stamp = ros::Time::now();
        track_map[track_id].header.frame_id = "/map";
        track_map[track_id].id = track_id;
        geometry_msgs::Pose pose; 
        

        pose.position.x = F_pub.p.x();
        pose.position.y = F_pub.p.y();
        pose.position.z = F_pub.p.z();

        F_pub.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        track_map[track_id].pose.push_back(pose);
        
        track_pub_.publish(track_map[track_id]);
    }

}

// publish twist
void cob_cartesian_trajectories::pubTwistMarkers(const ros::Duration pub_duration, const geometry_msgs::Twist &Twist, const KDL::Frame &F_current)
{
    if ((ros::Time::now() - pub_timer) >= pub_duration)
    {
        double color_mixer;
        double offset = 0.0;

        //setting up marker msg
        visualization_msgs::Marker twist_marker;
        twist_marker.header.frame_id = "/map";
        twist_marker.header.stamp = ros::Time::now();
        twist_marker.ns = "twist";
        twist_marker.id = pub_counter;
        twist_marker.type = visualization_msgs::Marker::ARROW;
        twist_marker.action = visualization_msgs::Marker::ADD;
        twist_marker.lifetime = ros::Duration();

        twist_marker.scale.x = 0.01;
        twist_marker.scale.y = 0.02;

        //calculate the color of the marker depending on twist size
        color_mixer = 11.0*sqrt(Twist.linear.x*Twist.linear.x + Twist.linear.y*Twist.linear.y + Twist.linear.z*Twist.linear.z);
        if (color_mixer > 1.0) offset = color_mixer - 1.0;
        color_mixer = color_mixer - offset;
        twist_marker.color.a = 1.0;
        twist_marker.color.g = 1.0 - color_mixer;
        twist_marker.color.r = color_mixer;

        //add two markers for start and end point of the arrow
        geometry_msgs::Point p;
        p.x = F_current.p.x();
        p.y = F_current.p.y();
        p.z = F_current.p.z();
        twist_marker.points.push_back(p);
        p.x = F_current.p.x() + Twist.linear.x;
        p.y = F_current.p.y() + Twist.linear.y;
        p.z = F_current.p.z() + Twist.linear.z;
        twist_marker.points.push_back(p);
        
        twist_pub_.publish(twist_marker);
        pub_timer = ros::Time::now();
        pub_counter++;
    }
}

//old function to publish trajectory markers
void cob_cartesian_trajectories::sendMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_values";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    for(unsigned int i=0; i<trajectory_points.size(); i++)
    {
        //ROS_INFO("line %f %f %f %f\n", iX1, iY1, iX2, iY2);
        marker.points.push_back(trajectory_points[i]);
    }
    //map_pub_.publish(marker);
}

//TOOLS
//
void cob_cartesian_trajectories::vector3dKDLToEigen(KDL::Vector &from, Eigen::Vector3d &to)
{
    to = Eigen::Vector3d(from.x(), from.y(), from.z());
}

void cob_cartesian_trajectories::vector3dEigenToKDL(Eigen::Vector3d &from, KDL::Vector &to)
{
    to = KDL::Vector(from[0], from[1], from[2]);
}


// to avoid the jump from positive to negative and the other way around 
// when crossing pi or -pi for roll and yaw angles and pi/2 and -pi/2 for pitch
double cob_cartesian_trajectories::unwrapRPY(std::string axis, double angle)
{
    double unwrapped_angle = 0.0;
    double fractpart, intpart;
    double discont = PI;    //point of discontinuity

    // FORMULA: phi(n)_adjusted = MODULO( phi(n) - phi(n-1) + PI, 2*PI) + phi(n-1) - PI
    
    // the pitch angle is only defined from -PI/2 to PI/2
    if (strncmp(&axis[strlen(axis.c_str())-6], "_pitch", 6) == 0)
        discont = PI/2;
    
    fractpart = modf(((angle - last_rpy_angles[axis] + discont) / (2*discont)), &intpart);    // modulo for double and getting only the fractal part of the division
    
    if (fractpart >= 0.0) // to get always the positive modulo
        unwrapped_angle = (fractpart)*(2*discont) - discont + last_rpy_angles[axis];
    else
        unwrapped_angle = (1+fractpart)*(2*discont) - discont + last_rpy_angles[axis];

    std::cout << "modulo: " << intpart << " + " << (fractpart) << "\n";
    
    std::cout << axis << " angle: " << angle << "\n";
    std::cout << "unwrapped " << axis << " angle: " << unwrapped_angle << "\n";
    last_rpy_angles[axis] = unwrapped_angle;
    return unwrapped_angle;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "cob_cartesian_trajectories");
    cob_cartesian_trajectories controller ;
    ros::spin();

    return 0;
}
