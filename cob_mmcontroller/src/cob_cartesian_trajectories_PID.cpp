#include <cob_mmcontroller/cob_cartesian_trajectories_PID.h>

cob_cartesian_trajectories::cob_cartesian_trajectories() : as_(n, "moveCirc", boost::bind(&cob_cartesian_trajectories::moveCircActionCB, this, _1), false), as2_(n, "moveLin", boost::bind(&cob_cartesian_trajectories::moveLinActionCB, this, _1), false), as_model_(n, "moveModel", boost::bind(&cob_cartesian_trajectories::moveModelActionCB, this, _1), false)
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
    as_.start();
    as2_.start();
    as_model_.start();
    targetDuration = 0;
    currentDuration = 0;
    mode = "prismatic";     // prismatic, rotational, trajectory, model
   
    pub_timer = ros::Time::now();
    pub_counter = 0;
    double const PI = 4.0*std::atan(1.0);

    getJointLimits(UpperLimits, LowerLimits);   // get arm joint limits from topic /robot_description

    tf::TransformBroadcaster br;
    
    bTrack_start = false;
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



void cob_cartesian_trajectories::moveCircActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal)
{
    mode = "circular";
    current_hinge = goal->hinge;
    if(start())
    {
        while(bRun)
        {
            //wait until finished
            sleep(1);
        }
        as_.setSucceeded();
    }
    return;

}
void cob_cartesian_trajectories::moveLinActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal)
{
    mode = "linear";
    if(start())
    {
        while(bRun)
        {
            //wait until finished
            sleep(1);
        }
        as2_.setSucceeded();
    }
    return;

}
// action for model 
void cob_cartesian_trajectories::moveModelActionCB(const cob_mmcontroller::ArticulationModelGoalConstPtr& goal)
{
    articulation_msgs::ModelMsg pub_model;
    //tf broadcaster 
    //static tf::TransformBroadcaster br;
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
        bTrack_start = true;
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
    //std::cout << "Twist x: " << 0.1 * F_diff.p.x() << " y: " << 0.1 * F_diff.p.y() << "\n";
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

/*//rotational trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;
    double rot_center_x;
    double rot_center_y;
    double rot_center_z;
    double rot_radius;
    KDL::Rotation temp_rot;
    
    double start_roll, start_pitch, start_yaw = 0.0;        //debug
    double target_roll, target_pitch, target_yaw = 0.0;     //debug

    // parameter from parameter server
    angle = getParamValue("action");
    rot_radius = getParamValue("rot_radius");
    rot_center_x = getParamValue("rot_center.x");
    rot_center_y = getParamValue("rot_center.y");
    rot_center_z = getParamValue("rot_center.z");

    // calculating target angle with respect to the time
    partial_angle = angle * (dt/targetDuration);
    
    // calculating rot_radius
    if (rot_radius == 0.0)
    {
        ROS_DEBUG("Parameter rot_radius no set and will be calculated!");
        rot_radius = sqrt(rot_center_x*rot_center_x + rot_center_y*rot_center_y); // + rot_center_z*rot_center_z);
    }
    
    // calulating target position with respect to the time and the start position
    F_target.p.x(F_EE_start.p.x() + rot_radius*sin(partial_angle));
    F_target.p.y(F_EE_start.p.y() + rot_radius*(1-cos(partial_angle)));
    F_target.p.z(F_EE_start.p.z()); // keep initial height
    //F_target.p.z(F_EE_start.p.z + rot_radius*sin(partial_angle));
    
    
    // calculating the target rotation (at the moment only around the z-axis)
    temp_rot = Rotation::Identity();    // TODO Rotation::Rot(vector,angle);
    std::cout << "temp_rot" << "\n" << temp_rot << "\n";
    temp_rot.DoRotZ(partial_angle);     // rotation axis of the articulation
    std::cout << "temp_rot" << "\n" << temp_rot << "\n";
    temp_rot = temp_rot*F_EE_start.M;      // rotation is composed of two rotations. 
                                        // first about the z-axis of the base_link frame and 
    std::cout << "temp_rot" << "\n" << temp_rot << "\n";
                                        // than the rotation from base_link to start frame of the gripper
    
    F_target.M = temp_rot;

    // ------------debugging output-----------------------
    cout << "rot_radius: " << rot_radius << "\n";
    cout << "Partial Angle: " << partial_angle << "\n";
    cout << "sin: " << sin(partial_angle) << "\n";
    cout << "cos: " << 1-cos(partial_angle) << "\n";

    std::cout << "F_X: " << F_target.p.x() << " F_Y: " << F_target.p.y() << " F_Z: " << F_target.p.z() << "\n";
    
    cout << "F_EE_start.M: " << "\n" << F_EE_start.M << "\n";
    cout << "F_target.M: " << "\n" << F_target.M << "\n";
    
    F_EE_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    cout << "Target R-P-Y: " << target_roll << " " << target_pitch << " " << target_yaw << "\n";

    cout << "Start R-P-Y: " << start_roll << " " << start_pitch << " " << start_yaw << "\n";
    cout << "Target R-P-Y: " << target_roll << " " << target_pitch << " " << target_yaw << "\n";
    cout << "Current Duration: " << dt << "\n";
}*/

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

//calculate start position of trajectory from rot_axis, rot_radius and gripper position
void cob_cartesian_trajectories::getRotStart(KDL::Frame &F_track_start)
{
    double rot_center_x;
    double rot_center_y;
    double rot_center_z;
    double rot_axis_x;
    double rot_axis_y;
    double rot_axis_z;
    double rot_axis_w;

    int axis_no;
    int index_maxCoeff;

    KDL::Frame F_articulation;
    
    Eigen::Vector3d articulation_Z;
    Eigen::Vector3d articulation_O;
    Eigen::Vector3d track_start_O;
    Eigen::Vector3d perpendicular;
    Eigen::Vector3d perpendicular_EE;
    Eigen::Vector3d trans_ee_art_EE;

    Eigen::Hyperplane<double, 3> door_plane;
    
    map<int, KDL::Vector> track_start_rot;

    //tf transform
    tf::Transform transform_track_start;

    rot_center_x = getParamValue("rot_center.x");
    rot_center_y = getParamValue("rot_center.y");
    rot_center_z = getParamValue("rot_center.z");
    rot_axis_x = getParamValue("rot_axis.x");
    rot_axis_y = getParamValue("rot_axis.y");
    rot_axis_z = getParamValue("rot_axis.z");
    rot_axis_w = getParamValue("rot_axis.w");
    
    // creating articulation frame TODO: check orientation
    F_articulation.p.x(rot_center_x);
    F_articulation.p.y(rot_center_y);
    F_articulation.p.z(rot_center_z);
    F_articulation.M = KDL::Rotation::Quaternion(rot_axis_x, rot_axis_y, rot_axis_z, rot_axis_w);    //TODO: check if z-axis = articulation axis
    debug ? (std::cout << "F_articulation" <<  F_articulation << "\n" ) : std::cout << ""; //debug

    // EE start position is sdh_tip_link frame
    std::cout << "F_EE_start" <<  F_EE_start << "\n"; //debug

    // origin of track start frame correlates with EE start
    F_track_start.p = F_EE_start.p;

    // TODO distance to articulation == rot_radius!?

    // z-axis of articulation frame in global coordinates to use as normal for rotation plane
    KDL::Vector rot_axis = F_articulation.M*KDL::Vector(0, 0, 1); // TODO check
    KDL::Vector articulation_Z_KDL = F_articulation.M.UnitZ(); // TODO check
    vector3dKDLToEigen(articulation_Z_KDL, articulation_Z);
    std::cout << "rot_axis" << "\n" <<  rot_axis << "\n"; //debug
    std::cout << "articulation_Z_KDL" << "\n" <<  articulation_Z_KDL << "\n"; //debug
    std::cout << "articulation_Z" << "\n" <<  articulation_Z << "\n"; //debug

    // origin of track start frame as point in plane of rotation
    vector3dKDLToEigen(F_track_start.p, track_start_O);
    std::cout << "track_start_O" << "\n" <<  track_start_O << "\n"; //debug

    // origin of articulation frame
    vector3dKDLToEigen(F_articulation.p, articulation_O);
    std::cout << "articulation_O" << "\n" <<  articulation_O << "\n"; //debug

    // door plane TODO change name
    door_plane = Eigen::Hyperplane<double, 3>::Through(articulation_O, articulation_O + articulation_Z, track_start_O);
    std::cout << "door_plane_offset" << "\n" <<  door_plane.offset() << "\n"; //debug
    std::cout << "door_plane_coeffs" << "\n" <<  door_plane.coeffs() << "\n"; //debug
    std::cout << "door_plane_distance to articulation_O" << "\n" <<  door_plane.absDistance(articulation_O) << "\n"; //debug
    std::cout << "door_plane_distance to track_start" << "\n" <<  door_plane.absDistance(track_start_O) << "\n"; //debug

    // perpendicular of articulation_Z through track_start_O TODO check if normalized (should be)
    perpendicular = articulation_Z.cross(door_plane.normal());
    std::cout << "perdendicular" << "\n" <<  perpendicular << "\n"; //debug
    if (perpendicular.norm() < 1e-6)
        ROS_ERROR("Normals are parallel");

    std::cout << "perdendicular norm" << "\n" <<  perpendicular.norm() << "\n"; //debug
    std::cout << "perpendicular normalized" << "\n" <<  perpendicular/perpendicular.norm() << "\n"; //debug

    // translation from EE start to articulation in global coord
    KDL::Vector trans_ee_art_KDL = F_articulation.p - F_EE_start.p;
    trans_ee_art_KDL.Normalize();
    std::cout << "trans_ee_art_KDL" << "\n" <<  trans_ee_art_KDL << "\n"; //debug

    // transform trans_ee_art into F_EE_start
    KDL::Vector trans_ee_art_KDL_EE = F_EE_start.M.Inverse()*trans_ee_art_KDL;
    std::cout << "trans_ee_art_KDL_EE" << "\n" <<  trans_ee_art_KDL_EE << "\n"; //debug

    // transform perpendicular into EE frame
    KDL::Vector perpendicular_EE_KDL = F_EE_start.M.Inverse()*KDL::Vector(perpendicular[0], perpendicular[1], perpendicular[2]);
    std::cout << "perpendicular_EE_KDL" << "\n" <<  perpendicular_EE_KDL << "\n"; //debug
    vector3dKDLToEigen(perpendicular_EE_KDL, perpendicular_EE);
    std::cout << "perpendicular_EE" << "\n" <<  perpendicular_EE << "\n"; //debug

    // get axis pointing on articulation
    vector3dKDLToEigen(trans_ee_art_KDL_EE, trans_ee_art_EE);
    trans_ee_art_EE.maxCoeff(&axis_no);  // index correlates with axis
    std::cout << "axis_no" << "\n" << axis_no << "\n"; //debug

    // get maximal coeff of vector
    double maxCoeff_perpendicular_EE = perpendicular_EE.maxCoeff(&index_maxCoeff);
    std::cout << "max perpendicular coeff" << "\n" <<  maxCoeff_perpendicular_EE << "\t" << index_maxCoeff << "\n"; //debug

    // check and set direction 
    if (sign(maxCoeff_perpendicular_EE) != sign(trans_ee_art_EE[index_maxCoeff]))
        perpendicular *= (-1.0);
        perpendicular_EE *= (-1.0);
    std::cout << "perpendicular" << "\n" <<  perpendicular << "\n"; //debug
    std::cout << "perpendicular_EE" << "\n" <<  perpendicular_EE << "\n"; //debug

    // collect vectors for F_track_start orientation
    vector3dEigenToKDL(perpendicular, track_start_rot[axis_no]);
    // check axis direction
    std::cout << "axis mod" << "\n" << axis_no%1 << "\n"; //debug
    KDL::Vector rot_axis_EE = F_EE_start*rot_axis;
    KDL::Vector articulation_Z_KDL_EE = F_EE_start*articulation_Z_KDL;
    if (rot_axis_EE.z() < 0.0)
        track_start_rot[abs(axis_no-1)] = (-1.0)*rot_axis;
    else
        track_start_rot[abs(axis_no-1)] = rot_axis;
    if (axis_no == 0)
    {
        Eigen::Vector3d track_start_Z = perpendicular.cross(articulation_Z);
        track_start_rot[2] = KDL::Vector(track_start_Z[0], track_start_Z[1], track_start_Z[2]);
    }   
    else if (axis_no == 1)
    {
        Eigen::Vector3d track_start_Z = articulation_Z.cross(perpendicular);
        track_start_rot[2] = KDL::Vector(track_start_Z[0], track_start_Z[1], track_start_Z[2]);
    }
    else
        ROS_ERROR("Wrong axis");
    std::cout << "rot vector x" << "\n" << track_start_rot[0] << "\n"; //debug
    std::cout << "rot vector y" << "\n" << track_start_rot[1] << "\n"; //debug
    std::cout << "rot vector z" << "\n" << track_start_rot[2] << "\n"; //debug
    F_track_start.M = KDL::Rotation(track_start_rot[0], track_start_rot[1], track_start_rot[2]);
    std::cout << "F_track_start" << "\n" << F_track_start << "\n"; //debug

    tf::TransformKDLToTF(F_track_start, transform_track_start);
    br.sendTransform(tf::StampedTransform(transform_track_start, ros::Time::now(), "/map", "/track_start"));
    
    bTrack_start = false;
}

//rotational 6D-trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;
    double rot_radius;
    /*double rot_center_x;
    double rot_center_y;
    double rot_center_z;
    double rot_axis_x;
    double rot_axis_y;
    double rot_axis_z;
    double rot_axis_w;
    KDL::Frame F_articulation;
    KDL::Frame F_EE_start;*/
    KDL::Frame F_track_start;
    KDL::Frame F_track;

    // get start frame of trajectory
    if (bTrack_start)
        getRotStart(F_track_start);
        
    angle = getParamValue("action");
    rot_radius = getParamValue("rot_radius");
    /*rot_center_x = getParamValue("rot_center.x");
    rot_center_y = getParamValue("rot_center.y");
    rot_center_z = getParamValue("rot_center.z");
    rot_axis_x = getParamValue("rot_axis.x");
    rot_axis_y = getParamValue("rot_axis.y");
    rot_axis_z = getParamValue("rot_axis.z");
    rot_axis_w = getParamValue("rot_axis.w");*/

    // calculating partial_angle
    partial_angle = angle * (dt/targetDuration);



    // creating trajectory frame w.r.t. the articulation frame
    // orientation is like sdh_tip_link frame
    F_track.p.x(rot_radius*cos(partial_angle));
    F_track.p.y(rot_radius*sin(partial_angle));

    F_track.M.RPY(-PI/2.0, -(PI/2.0+partial_angle), 0.0);  //TODO: check orientation //depending also on handle orientation

    // transformation of trajectory frame into base_link frame ??? map frame
    F_target = F_EE_start;//F_track*F_articulation;       // FT_a*FA_bl

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
void cob_cartesian_trajectories::vector3dKDLToEigen(KDL::Vector &from, Eigen::Vector3d &to)
{
    to = Eigen::Vector3d(from.x(), from.y(), from.z());
}

void cob_cartesian_trajectories::vector3dEigenToKDL(Eigen::Vector3d &from, KDL::Vector &to)
{
    to = KDL::Vector(from[0], from[1], from[2]);
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "cob_cartesian_trajectories");
    cob_cartesian_trajectories controller ;
    ros::spin();

    return 0;
}
