#include <ros/ros.h>
#include <cob_lookat_action/cob_lookat_action_server.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lookat_action");

    CobLookAtAction *lookat = new CobLookAtAction("lookat_action");
    if(lookat->init())
    {
        ROS_INFO_STREAM("lookat_action running...");
        ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize lookat_action");
    }

    return 0;
}
