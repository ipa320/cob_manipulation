#include <cob_obstacle_distance_moveit/obstacle_distance_moveit.h>

class CreateCollisionWorld : public collision_detection::CollisionWorldFCL
{
public:
    CreateCollisionWorld(const collision_detection::WorldPtr &world) :
            CollisionWorldFCL(world)
    {}

    void getCollisionObject(std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj)
    {
        std::map<std::string, collision_detection::FCLObject>::iterator it = fcl_objs_.begin();
        obj.reserve(fcl_objs_.size());

        for (it; it != fcl_objs_.end(); ++it)
        {
            obj.push_back(it->second.collision_objects_[0]);
        }
    }
};

class CreateCollisionRobot : public collision_detection::CollisionRobotFCL
{
public:
    CreateCollisionRobot(const robot_model::RobotModelConstPtr &model) :
            CollisionRobotFCL(model)
    {}

    void getCollisionObject(const robot_state::RobotState &state,
                            std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj)
    {
        collision_detection::FCLObject fcl_obj;
        constructFCLObject(state, fcl_obj);
        obj = fcl_obj.collision_objects_;
    }
};

void ObstacleDistanceMoveit::updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;
    robot_state::RobotState robot_state(planning_scene_ptr->getCurrentState());

    CreateCollisionWorld collision_world(planning_scene_ptr->getWorldNonConst());
    collision_world.getCollisionObject(world_obj);

    CreateCollisionRobot collision_robot(robot_state.getRobotModel());
    collision_robot.getCollisionObject(robot_state, robot_obj);

    this->robot_links_.clear();
    this->collision_objects_.clear();

    for (unsigned int i = 0; i < robot_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData *robot_link =
                static_cast<const collision_detection::CollisionGeometryData *>(robot_obj[i]->collisionGeometry()->getUserData());
        if (std::find(skip_links_.begin(), skip_links_.end(), robot_link->getID()) == skip_links_.end())
        {
            this->robot_links_[robot_link->getID()] = robot_obj[i];
        }
    }

    for (unsigned int i = 0; i < world_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData *collision_object =
                static_cast<const collision_detection::CollisionGeometryData *>(world_obj[i]->collisionGeometry()->getUserData());
        this->collision_objects_[collision_object->getID()] = world_obj[i];
    }
}

void ObstacleDistanceMoveit::planningSceneTimerCallback(const ros::TimerEvent& event)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    moveit_msgs::PlanningScene scene;
    planning_scene_ptr->getPlanningSceneMsg(scene);
    monitored_scene_pub_.publish(scene);
}

bool ObstacleDistanceMoveit::planningSceneCallback(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    planning_scene_ptr->getPlanningSceneMsg(res.scene, req.components);

    return true;
}

void ObstacleDistanceMoveit::calculateDistanceTimerCallback(const ros::TimerEvent& event)
{
    distance_pub_.publish(getObstacleDistances());
}

bool ObstacleDistanceMoveit::calculateDistanceServiceCallback(cob_control_msgs::GetObstacleDistance::Request &req,
                                                              cob_control_msgs::GetObstacleDistance::Response &resp)
{
    resp.result = getObstacleDistances();
    return true;
}

cob_control_msgs::ObstacleDistances ObstacleDistanceMoveit::getObstacleDistances()
{
    cob_control_msgs::ObstacleDistances distance_infos;
    ros::Time now = ros::Time::now();

    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links = this->robot_links_;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_self_links = this->robot_links_;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects = this->collision_objects_;

    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    std::string planning_frame = planning_scene_ptr->getPlanningFrame();

    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator link_it;
    for (link_it = robot_links.begin(); link_it!=robot_links.end(); ++link_it)
    {
        std::string link_name = link_it->first;

        const boost::shared_ptr<fcl::CollisionObject> link_object = robot_links[link_name];
        ROS_DEBUG_STREAM("RobotLink: " << link_name << ", Type: " << link_object->getObjectType());

        std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator obj_it;
        for (obj_it = collision_objects.begin(); obj_it != collision_objects.end(); ++obj_it)
        {
            std::string obj_name = obj_it->first;
            const boost::shared_ptr<fcl::CollisionObject> obj_object = collision_objects[obj_name];
            ROS_DEBUG_STREAM("CollisionLink: " << obj_name << ", Type: " << obj_object->getObjectType());

            if(obj_object->getObjectType() == fcl::OT_OCTREE)
            {
                ROS_WARN_THROTTLE(1, "Consideration of <octomap> not yet implemented");
                continue;
            }

            cob_control_msgs::ObstacleDistance info;
            info = getObstacleDistance(link_object, obj_object);

            info.header.frame_id = planning_frame;
            info.header.stamp = now;
            info.link_of_interest = link_name;
            info.obstacle_id = obj_name;

            distance_infos.distances.push_back(info);
        }

        std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator self_it;
        robot_self_links.erase(link_name);
        for (self_it = robot_self_links.begin(); self_it != robot_self_links.end(); ++self_it)
        {
            std::string self_name = self_it->first;
            collision_detection::AllowedCollision::Type type;

            if(acm_.getEntry(link_name, self_name, type))
            {
                if(type != collision_detection::AllowedCollision::ALWAYS)
                {
                    const boost::shared_ptr<fcl::CollisionObject> self_object = robot_self_links[self_name];
                    ROS_DEBUG_STREAM("CollisionLink: " << self_name << ", Type: " << self_object->getObjectType());

                    cob_control_msgs::ObstacleDistance info;
                    info = getObstacleDistance(link_object, self_object);

                    info.header.frame_id = planning_frame;
                    info.header.stamp = now;
                    info.link_of_interest = link_name;
                    info.obstacle_id = self_name;

                    distance_infos.distances.push_back(info);
                }
                else
                {
                    ROS_DEBUG_STREAM("CollisionLink: " << self_name << ", Allowed: " << type);
                }
            }
        }
    }
    return distance_infos;
}

cob_control_msgs::ObstacleDistance ObstacleDistanceMoveit::getObstacleDistance(const boost::shared_ptr<fcl::CollisionObject> object_a,
                                                                               const boost::shared_ptr<fcl::CollisionObject> object_b)
{
    fcl::DistanceRequest req(true);  // enable_nearest_points
    fcl::DistanceResult res;
    res.update(MAXIMAL_MINIMAL_DISTANCE, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE);

    Eigen::Vector3d np_object_a;
    Eigen::Vector3d np_object_b;

    double dist = fcl::distance(object_a.get(), object_b.get(), req, res);

    // this is to prevent what seems to be a nasty bug in fcl
    if(object_a->getObjectType() == fcl::OT_GEOM && object_b->getObjectType() == fcl::OT_BVH)
    {
        // res.nearest_points is swapped in this case
        np_object_a(0) = res.nearest_points[1][0];
        np_object_a(1) = res.nearest_points[1][1];
        np_object_a(2) = res.nearest_points[1][2];

        np_object_b(0) = res.nearest_points[0][0];
        np_object_b(1) = res.nearest_points[0][1];
        np_object_b(2) = res.nearest_points[0][2];
    }
    else
    {
        np_object_a(0) = res.nearest_points[0][0];
        np_object_a(1) = res.nearest_points[0][1];
        np_object_a(2) = res.nearest_points[0][2];

        np_object_b(0) = res.nearest_points[1][0];
        np_object_b(1) = res.nearest_points[1][1];
        np_object_b(2) = res.nearest_points[1][2];
    }
    // ToDo: are there other cases? OT_OCTREE? see fcl::OBJECT_TYPE

    geometry_msgs::Vector3 np_object_a_msg;
    tf::vectorEigenToMsg(np_object_a, np_object_a_msg);
    ROS_DEBUG_STREAM("NearestPoint OBJ_A: \n" << np_object_a_msg);

    geometry_msgs::Vector3 np_object_b_msg;
    tf::vectorEigenToMsg(np_object_b, np_object_b_msg);
    ROS_DEBUG_STREAM("NearestPoint OBJ_B: \n" << np_object_b_msg);

    // Transformation for object_a
    fcl::Transform3f fcl_trans_object_a = object_a->getTransform();
    fcl::Vec3f fcl_vec_object_a = fcl_trans_object_a.getTranslation();
    fcl::Quaternion3f fcl_quat_object_a = fcl_trans_object_a.getQuatRotation();

    Eigen::Affine3d eigen_trans_object_a;
    tf::Transform tf_trans_object_a(tf::Quaternion(fcl_quat_object_a.getX(),fcl_quat_object_a.getY(),fcl_quat_object_a.getZ(),fcl_quat_object_a.getW()),
                                    tf::Vector3(fcl_vec_object_a[0],fcl_vec_object_a[1],fcl_vec_object_a[2]));
    tf::transformTFToEigen(tf_trans_object_a, eigen_trans_object_a);

    geometry_msgs::Transform trans_object_a_msg;
    tf::transformTFToMsg(tf_trans_object_a, trans_object_a_msg);
    ROS_DEBUG_STREAM("Transform OBJ_A: \n" << trans_object_a_msg);

    // Transformation for object_b
    fcl::Transform3f fcl_trans_object_b = object_b->getTransform();
    fcl::Vec3f fcl_vec_object_b = fcl_trans_object_b.getTranslation();
    fcl::Quaternion3f fcl_quat_object_b = fcl_trans_object_b.getQuatRotation();

    Eigen::Affine3d eigen_trans_object_b;
    tf::Transform tf_trans_object_b(tf::Quaternion(fcl_quat_object_b.getX(),fcl_quat_object_b.getY(),fcl_quat_object_b.getZ(),fcl_quat_object_b.getW()),
                                    tf::Vector3(fcl_vec_object_b[0],fcl_vec_object_b[1],fcl_vec_object_b[2]));
    tf::transformTFToEigen(tf_trans_object_b, eigen_trans_object_b);

    geometry_msgs::Transform trans_object_b_msg;
    tf::transformTFToMsg(tf_trans_object_b, trans_object_b_msg);
    ROS_DEBUG_STREAM("Transform OBJ_B: \n" << trans_object_b_msg);

    //  in case both objects are of OBJECT_TYPE OT_BVH the nearest points are already given in PlanningFrame coordinates
    if(!(object_a->getObjectType() == fcl::OT_BVH && object_b->getObjectType() == fcl::OT_BVH))
    {
        np_object_a = eigen_trans_object_a * np_object_a;
        np_object_b = eigen_trans_object_b * np_object_b;
    }
    // ToDo: are there other cases? OT_OCTREE? see fcl::OBJECT_TYPE

    if (dist < 0) dist = 0;

    cob_control_msgs::ObstacleDistance info;
    info.distance = dist;

    tf::vectorEigenToMsg(np_object_a, info.nearest_point_frame_vector);
    tf::vectorEigenToMsg(np_object_b, info.nearest_point_obstacle_vector);    
    ROS_DEBUG_STREAM("NearestPointTransformed OBJ_A: \n" << info.nearest_point_frame_vector);
    ROS_DEBUG_STREAM("NearestPointTransformed OBJ_B: \n" << info.nearest_point_obstacle_vector);

    return info;
}

ObstacleDistanceMoveit::ObstacleDistanceMoveit()
{
    MAXIMAL_MINIMAL_DISTANCE = 1.0; //m
    double update_frequency = 50.0; //Hz

    std::string robot_description = "/robot_description";
    std::string robot_description_semantic = "/robot_description_semantic";
    std::string distance_topic = "/obstacle_distances";
    std::string distance_service = "/calculate_distances";

    // Get AllowedCollisionMatrix
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description", "robot_description_semantic");
    planning_scene::PlanningScene pss(robot_model_loader.getURDF(), robot_model_loader.getSRDF());
    acm_ = pss.getAllowedCollisionMatrix();

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description, tf_listener_);

    planning_scene_monitor_->setStateUpdateFrequency(update_frequency);
    planning_scene_monitor_->startSceneMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC);
    planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                                       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                                       true);  // load_octomap_monitor
    planning_scene_monitor_->startStateMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC,
                                               planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC);
    planning_scene_monitor_->addUpdateCallback(boost::bind(&ObstacleDistanceMoveit::updatedScene, this, _1));

    skip_links_.clear();
    ros::NodeHandle("~").getParam("skip_links", skip_links_);

    distance_timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency), &ObstacleDistanceMoveit::calculateDistanceTimerCallback, this);
    distance_pub_ = nh_.advertise<cob_control_msgs::ObstacleDistances>(distance_topic, 1);
    distance_server_ = nh_.advertiseService(distance_service, &ObstacleDistanceMoveit::calculateDistanceServiceCallback, this);

    planning_scene_timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency), &ObstacleDistanceMoveit::planningSceneTimerCallback, this);
    monitored_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/monitored_planning_scene", 1);
    monitored_scene_server_ = nh_.advertiseService("/get_planning_scene", &ObstacleDistanceMoveit::planningSceneCallback, this);
}
