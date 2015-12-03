#include <obstacle_distance/obstacle_distance.h>

class CreateCollisionWorld : public collision_detection::CollisionWorldFCL {
public:
    CreateCollisionWorld(const collision_detection::WorldPtr &world) :
            CollisionWorldFCL(world) {
    }

    void getCollisionObject(std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj) {
        std::map<std::string, collision_detection::FCLObject>::iterator it = fcl_objs_.begin();
        obj.reserve(fcl_objs_.size());

        for (it; it != fcl_objs_.end(); ++it) {
            obj.push_back(it->second.collision_objects_[0]);
        }
    }
};

class CreateCollisionRobot : public collision_detection::CollisionRobotFCL {
public:
    CreateCollisionRobot(const robot_model::RobotModelConstPtr &model) :
            CollisionRobotFCL(model) {
    }

    void getCollisionObject(const robot_state::RobotState &state,
                            std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj) {
        collision_detection::FCLObject fcl_obj;
        constructFCLObject(state, fcl_obj);
        obj = fcl_obj.collision_objects_;
    }
};

void ObstacleDistance::updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type) {

    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    CreateCollisionWorld collision_world(planning_scene_ptr->getWorldNonConst());
    robot_state::RobotState robot_state(planning_scene_ptr->getCurrentState());

    CreateCollisionRobot collision_robot(robot_state.getRobotModel());
    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;

    collision_robot.getCollisionObject(robot_state, robot_obj);
    collision_world.getCollisionObject(world_obj);

    robot_links_list.clear();
    collision_objects_list.clear();
    kinematic_list.empty();

    for (int i = 0; i < robot_obj.size(); i++) {
        const collision_detection::CollisionGeometryData *robot_link =
                static_cast<const collision_detection::CollisionGeometryData *>(robot_obj[i]->collisionGeometry()->getUserData());
        robot_links_list[robot_link->getID()] = robot_obj[i];
        kinematic_list.push_back(robot_link->getID());
    }
    for (int i = 0; i < world_obj.size(); i++) {
        const collision_detection::CollisionGeometryData *collision_object =
                static_cast<const collision_detection::CollisionGeometryData *>(world_obj[i]->collisionGeometry()->getUserData());
        collision_objects_list[collision_object->getID()] = world_obj[i];
    }
}

bool ObstacleDistance::calculateDistance(obstacle_distance::GetObstacleDistance::Request &req,
                                         obstacle_distance::GetObstacleDistance::Response &resp) {
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links_list = this->robot_links_list;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects_list = this->collision_objects_list;
    std::vector<std::string> kinematic_list = this->kinematic_list;
    if (req.links.size() == 2) {
        bool start = false;
        for (int i = 0; i < kinematic_list.size(); i++) {
            if (!start && kinematic_list[i] == req.links[0]) start = true;
            if (start) {
                if (req.objects.size() == 0) {
                    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator it;
                    for (it = collision_objects_list.begin(); it != collision_objects_list.end(); ++it) {
                        resp.link_to_object.push_back(kinematic_list[i] + "_to_" + it->first);
                        resp.distances.push_back(
                                ObstacleDistance::getMinimalDistance(kinematic_list[i], it->first, robot_links_list,
                                                                     collision_objects_list));
                    }
                } else {
                    for (int y = 0; y < req.objects.size(); y++) {
                        resp.link_to_object.push_back(kinematic_list[i] + " to " + req.objects[y]);
                        resp.distances.push_back(ObstacleDistance::getMinimalDistance(kinematic_list[i], req.objects[y],
                                                                                      robot_links_list,
                                                                                      collision_objects_list));
                    }
                }
                if (kinematic_list[i] == req.links[1]) break;
            }
        }
    } else {
        if (req.objects.size() == 0) {
            std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator it;
            for (it = collision_objects_list.begin(); it != collision_objects_list.end(); ++it) {
                resp.link_to_object.push_back(req.links[0] + "_to_" + it->first);
                resp.distances.push_back(ObstacleDistance::getMinimalDistance(req.links[0], it->first, robot_links_list,
                                                                              collision_objects_list));
            }
        } else {
            for (int y = 0; y < req.objects.size(); y++) {
                resp.link_to_object.push_back(req.links[0] + " to " + req.objects[y]);
                resp.distances.push_back(
                        ObstacleDistance::getMinimalDistance(req.links[0], req.objects[y], robot_links_list,
                                                             collision_objects_list));
            }
        }
    }
    return true;
}

double ObstacleDistance::getMinimalDistance(std::string robot_link_name,
                                            std::string collision_object_name,
                                            std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links,
                                            std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects) {
    fcl::DistanceResult res;
    res.update(MAXIMAL_MINIMAL_DISTANCE, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE);

    double dist = fcl::distance(robot_links[robot_link_name].get(),
                                collision_objects[collision_object_name].get(),
                                fcl::DistanceRequest(),
                                res);
    if (dist < 0) dist = 0;
    return dist;
}

ObstacleDistance::ObstacleDistance()
        : ros::NodeHandle() {
    MAXIMAL_MINIMAL_DISTANCE = 5.0; //m
    double update_frequency = 100.0; //Hz
    bool error = false;

    std::string robot_description;
    std::vector<std::string> distance_service;
    getParam(ros::this_node::getName() + "/obstacle_distance/robot_description", robot_description);
    getParam(ros::this_node::getName() + "/obstacle_distance/services", distance_service);
    if (robot_description == "" || distance_service[0] == "") error = true;

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    try {
        planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description,
                                                                                                   tf_listener_);
    } catch (ros::InvalidNameException) {
        error = true;
    }

    calculate_obstacle_distance_ = advertiseService(distance_service[0], &ObstacleDistance::calculateDistance, this);

    if (!error) {
        planning_scene_monitor_->setStateUpdateFrequency(update_frequency);
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->addUpdateCallback(boost::bind(&ObstacleDistance::updatedScene, this, _1));

        ROS_INFO("obstacle_distance_node: Node started!");
    } else
        ROS_ERROR("obstacle_distance_node: Node failed!");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_distance_node");

    ObstacleDistance ob;
    ros::spin();

    ros::shutdown();
    return 0;
}
