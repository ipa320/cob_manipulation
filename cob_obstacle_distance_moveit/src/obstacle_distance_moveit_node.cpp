#include <cob_obstacle_distance_moveit/obstacle_distance_moveit.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_distance_node");

    ObstacleDistanceMoveit odm;
    ros::spin();
}
