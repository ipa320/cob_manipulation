#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cob_srvs/Trigger.h>
#include <deque>
#include <tf/transform_listener.h>

ros::Publisher twist_pub_;
ros::ServiceServer serv;
tf::TransformListener *tflistener;
std::deque<double> x_cache;
std::deque<double> y_cache;
std::deque<double> z_cache;
int num_cache = 20;
bool bRunning = false;
double offset = -11.5;

bool TwistInputTrigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(!bRunning)
	{
		bRunning = true;
		ROS_INFO("Starting wrench input");
	}
	else
	{
		bRunning = false;
		ROS_INFO("Stopping wrench input");
	}	
	return true;
} 

void spnavCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
if(bRunning)
{
  geometry_msgs::Vector3Stamped force_wrench;
  force_wrench.header.frame_id = msg->header.frame_id;
  force_wrench.vector = msg->wrench.force;
  geometry_msgs::Vector3Stamped force_bl;
  try{
    tflistener->transformVector("base_link", force_wrench, force_bl);

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  std::cout << "Force Vector : " << force_bl.vector.x << " " << force_bl.vector.y << " " << force_bl.vector.z << "\n";

  double average_x = 0.0;
  double average_y = 0.0;
  double average_z = 0.0;
  if(num_cache == 0)
  {
    x_cache.pop_front();
    y_cache.pop_front();
    z_cache.pop_front();
  }
  else
    {
      num_cache--;
    }

  x_cache.push_back(force_bl.vector.x);
  y_cache.push_back(force_bl.vector.y);
  z_cache.push_back(force_bl.vector.z + offset);
  for(unsigned int i = 0; i < z_cache.size(); i++)
    {
      average_x += x_cache.at(i);
      average_y += y_cache.at(i);
      average_z += z_cache.at(i);
    }
  average_x /= x_cache.size();
  average_y /= y_cache.size();
  average_z /= z_cache.size();

	geometry_msgs::Twist new_twist;
	
	if(average_x < -2.0)
	    new_twist.linear.x = -0.02 * average_x;
	else
	  {
	    if(average_x > 2.0)
	      new_twist.linear.x = -0.02 * average_x;
	    else
	      new_twist.linear.x = 0.0;
	  }
	/*
	if(average_y < -2.0)
	    new_twist.linear.y = 0.02 * average_y;
	else
	  {
	    if(average_y > 2.0)
	      new_twist.linear.y = 0.02 * average_y;
	    else
	      new_twist.linear.y = 0.0;
	  }
	if(average_z < -0.2)
	    new_twist.linear.z = 0.02 * average_z;
	else
	  {
	    if(average_z > 0.2)
	      new_twist.linear.z = 0.02 * average_z;
	    else
	      new_twist.linear.z = 0.0;
	  }
	*/
	std::cout << "Sending twist of " << new_twist.linear.z << " current force is " << average_z << "\n";
	twist_pub_.publish(new_twist);
}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "cob_spnav_filter");
	ros::NodeHandle n;
	tflistener = new tf::TransformListener();
	ros::Subscriber spnav = n.subscribe("/arm_controller/wrench", 1, spnavCallback);
	serv = n.advertiseService("/mm/twist_input_run", TwistInputTrigger);
	twist_pub_ = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
	ros::spin();
	return 0;
}
