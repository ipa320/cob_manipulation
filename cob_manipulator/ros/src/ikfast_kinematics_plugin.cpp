
#include "ros/ros.h"
#include <kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <iostream>
#include <algorithm>
#define IKFAST_NO_MAIN
#ifdef IKFAST_REAL
typedef IKFAST_REAL IKReal;
#else
typedef double IKReal;
#endif


namespace cob3_arm_kinematics
{ 

class ik_solver_base{
public:
    virtual int solve(KDL::Frame &pose_frame, const std::vector<double> &ik_seed_state) = 0;
    virtual void getSolution(int i, std::vector<double> &solution) = 0;
    virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution) = 0;
};

template <class  T> class ikfast_solver: public ik_solver_base{
public:
    typedef bool (*ik_type)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<T>& vsolutions);
    ikfast_solver(ik_type ik,int numJoints):ik(ik),numJoints(numJoints) {}
    virtual int solve(KDL::Frame &pose_frame, const std::vector<double> &vfree){
      
      solutions.clear();
      ik(pose_frame.p.data, pose_frame.M.data, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      
      return solutions.size();
    }
    virtual void getSolution(int i, std::vector<double> &solution){
      solution.clear();
      std::vector<IKReal> vsolfree(solutions[i].GetFree().size());
      solution.clear();
      solution.resize(numJoints);
      solutions[i].GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
      //std::cout << "solution " << i << ":" ;
      //for(int j=0;j<numJoints; ++j)
      //	  std::cout << " " << solution[j];
      //std::cout << std::endl;
	  
      //ROS_ERROR("%f %d",solution[2],vsolfree.size());
    }
    double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
    	double dist_sqr = 0;
    	for(size_t i=0; i< ik_seed_state.size(); ++i){
			if(fabs(solution[i] - ik_seed_state[i]) > 2*M_PI){ 
				if(ik_seed_state[i] < 0){
					if(solution[i] > 0)	 solution[i] -= 2*M_PI;
				}else{
					if(solution[i] < 0) solution[i] += 2*M_PI;
				}
			}

			dist_sqr += fabs(solution[i] - ik_seed_state[i]);
    	}
    	return dist_sqr;
    }

    double harmonize_old(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
    	double dist_sqr = 0;
    	for(size_t i=0; i< ik_seed_state.size(); ++i){
    		double diff = ik_seed_state[i] - solution[i];
    		if( diff > M_PI ) solution[i]+=2*M_PI; 
    		else if (diff < -M_PI) solution[i]-=2*M_PI;
			diff = ik_seed_state[i] - solution[i];
			dist_sqr += fabs(diff);
    	}
    	return dist_sqr;
    }
    virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
      double mindist = 0;
      int minindex = -1;
      std::vector<double> sol;
      for(size_t i=0;i<solutions.size();++i){
		  getSolution(i,sol);
		  double dist = harmonize(ik_seed_state, sol);
		  //std::cout << "dist[" << i << "]= " << dist << std::endl;
		  if(minindex == -1 || dist<mindist){
			minindex = i;
			mindist = dist;
		  }
      }
      if(minindex >= 0) getSolution(minindex,solution);
    }
    
    
    
private:
    ik_type ik;
    std::vector<T> solutions;
    int numJoints;
};

//Namespace for cob3-2
namespace cob3_2{
#include "ikfast_cob3_2.cpp"
}

//Namespace for cob3-3
namespace cob3_3{
#include "ikfast_cob3_3.cpp"
}

//Namespace for lwa_sdh
namespace lwa_ext{
#include "ikfast_lwa_ext.cpp"
}

class IKFastKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joints;
  std::vector<double> jointMin;
  std::vector<double> jointMax;
  std::vector<std::string> links;
  std::string tip_name, root_name;
  ik_solver_base* ik_solver;
  size_t numJoints;
  std::vector<int> freeParams;
  void (*fk)(const IKReal* j, IKReal* eetrans, IKReal* eerot);
  
public:

    IKFastKinematicsPlugin():ik_solver(0) {}
    ~IKFastKinematicsPlugin(){ if(ik_solver) delete ik_solver;}

  void fillFreeParams(int count, int *array) { freeParams.clear(); for(int i=0; i<count;++i) freeParams.push_back(array[i]); }
  
  bool initialize(std::string name) {

      ros::NodeHandle node_handle("~/"+name);

      std::string robot;
      node_handle.param("robot",robot,std::string());
      
      if(robot == "cob3-2"){
	fillFreeParams(cob3_2::getNumFreeParameters(),cob3_2::getFreeParameters());
	numJoints = cob3_2::getNumJoints();
	ik_solver = new ikfast_solver<cob3_2::IKSolution>(cob3_2::ik, numJoints);
	fk=cob3_2::fk;
      }else if(robot == "cob3-3"){
	fillFreeParams(cob3_3::getNumFreeParameters(),cob3_3::getFreeParameters());
	numJoints = cob3_3::getNumJoints();
	ik_solver = new ikfast_solver<cob3_3::IKSolution>(cob3_3::ik, numJoints);
	fk=cob3_3::fk;
      }else if(robot == "lwa_sdh"){
	fillFreeParams(lwa_ext::getNumFreeParameters(),lwa_ext::getFreeParameters());
	numJoints = lwa_ext::getNumJoints();
	ik_solver = new ikfast_solver<lwa_ext::IKSolution>(lwa_ext::ik, numJoints);
	fk=lwa_ext::fk;
      }else{
	ROS_FATAL("Robot '%s' unknown!",robot.c_str());
	return false;
      }
      ROS_INFO("Robot is '%s'!",robot.c_str());
      
      if(freeParams.size()>1){
	ROS_FATAL("Only one free joint paramter supported!");
	return false;
      }
      
      
      urdf::Model robot_model;
      std::string xml_string;

      std::string urdf_xml,full_urdf_xml;
      node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
      node_handle.searchParam(urdf_xml,full_urdf_xml);
      
      ROS_DEBUG("Reading xml file from parameter server\n");
      if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
	ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
	return false;
      }

      if (!node_handle.getParam("root_name", root_name)){
	ROS_FATAL("PR2IK: No root name found on parameter server");
	return false;
      }
      if (!node_handle.getParam("tip_name", tip_name)){
	ROS_FATAL("PR2IK: No tip name found on parameter server");
	return false;
      }
      
      node_handle.param(full_urdf_xml,xml_string,std::string());
      robot_model.initString(xml_string);

      boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(tip_name));
      while(link->name != root_name && joints.size() <= numJoints){
	//ROS_ERROR("link %s",link->name.c_str());
	links.push_back(link->name);
	boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
	if(joint && joint->type != urdf::Joint::FIXED){
	  //ROS_ERROR("joint %s",joint->name.c_str());
	  joints.push_back(joint->name);
	  if(joint->limits){
	    jointMin.push_back(joint->limits->lower);
	    jointMax.push_back(joint->limits->upper);
	  }
	}else{
	  ROS_WARN("no joint corresponding to %s",link->name.c_str());
   	}
	link = link->getParent();
      }
      
      if(joints.size() != numJoints){
	ROS_FATAL("Joints number mismatch.");
	return false;
      }
      
      links.push_back(root_name);
      
      std::reverse(links.begin(),links.end());
      std::reverse(joints.begin(),joints.end());
      std::reverse(jointMin.begin(),jointMin.end());
      std::reverse(jointMax.begin(),jointMax.end());
      
      for(size_t i=0; i <numJoints; ++i)
	ROS_INFO("%s %f %f",joints[i].c_str(),jointMin[i],jointMax[i]);
	
     return true;
    }

    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
		       int &error_code) {
	//ROS_ERROR("getPositionIK");
	
	std::vector<double> vfree(freeParams.size());
	for(std::size_t i = 0; i < freeParams.size(); ++i){
	    int p = freeParams[i];
//	    ROS_ERROR("%u is %f",p,ik_seed_state[p]);
	    vfree[i] = ik_seed_state[p];
	}

	KDL::Frame frame;
	tf::PoseMsgToKDL(ik_pose,frame);

	int numsol = ik_solver->solve(frame,vfree);

		
	if(numsol){
	  ik_solver->getClosestSolution(ik_seed_state,solution);
	  //ROS_ERROR("Solved!");
	  error_code = kinematics::SUCCESS;
	  return true;
	}
	
	error_code = kinematics::NO_IK_SOLUTION; 
	return false;
    }
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          const double &timeout,
                          std::vector<double> &solution,
			  int &error_code) {
  
	//ROS_ERROR("searchPositionIK1");
	if(freeParams.size()==0){
	  return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
	}
	
	KDL::Frame frame;
	tf::PoseMsgToKDL(ik_pose,frame);

	std::vector<double> vfree(freeParams.size());

	ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
	const double increment = 0.01; //TODO: make it a plugin parameter
	int counter = 0;

	vfree[0] = ik_seed_state[freeParams[0]];

	do{
	    int numsol = ik_solver->solve(frame,vfree);
	    //ROS_INFO("%f",vfree[0]);
	    
	    if(numsol > 0){
		ik_solver->getClosestSolution(ik_seed_state,solution);
		//ROS_ERROR("Solved!");
		error_code = kinematics::SUCCESS;
		return true;
	    }
	    int i = 0;
	    do{
	      if( i >= 2){
			error_code = kinematics::NO_IK_SOLUTION; 
			return false;
	      }
	      counter = counter > 0? - counter: 1 - counter;
	      vfree[0] = ik_seed_state[freeParams[0]] + increment*counter;
	      //std::cout << vfree[0] << " " << counter << " " << i << "  "  << jointMin[freeParams[0]] << " " << jointMax[freeParams[0]] << std::endl;
	      ++i;
	    }while( vfree[0] < jointMin[freeParams[0]] ||  vfree[0] > jointMax[freeParams[0]] );
	    
	}while (ros::Time::now() < maxTime && vfree[0] >= jointMin[freeParams[0]] &&  vfree[0] <= jointMax[freeParams[0]]);
	error_code = kinematics::NO_IK_SOLUTION; 
 	return false;
    }      
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          const double &timeout,
                          std::vector<double> &solution,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
			  int &error_code){
      
	//ROS_ERROR("searchPositionIK2");
	if(freeParams.size()==0){ // TODO: not closest, but valid solution!
	  return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
	}
	
	if(!desired_pose_callback.empty())
	  desired_pose_callback(ik_pose,ik_seed_state,error_code);
	if(error_code < 0)
	{
	  ROS_ERROR("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
	  return false;
	}
	KDL::Frame frame;
	tf::PoseMsgToKDL(ik_pose,frame);

	std::vector<double> vfree(freeParams.size());

	ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
	const double increment = 0.01; //TODO: make it a plugin parameter
	int counter = 0;

	std::vector<double> sol;
	vfree[0] = ik_seed_state[freeParams[0]];
	do{
	    int numsol = ik_solver->solve(frame,vfree);
	    
	    if(numsol > 0){
		if(solution_callback.empty()){
		    ik_solver->getClosestSolution(ik_seed_state,solution);
		    error_code = kinematics::SUCCESS;
		    return true;
		}

		for (int s = 0; s < numsol; ++s){
		    ik_solver->getSolution(s,sol);
		    solution_callback(ik_pose,sol,error_code);
		    
		    if(error_code == kinematics::SUCCESS){
		      solution = sol;
		      return true;
		    }
		}
	    }
	    int i = 0;
	    do{
	      if( i >= 2){
		error_code = kinematics::NO_IK_SOLUTION; 
		return false;
	      }
	      counter = counter > 0? - counter: 1 - counter;
	      vfree[0] = ik_seed_state[freeParams[0]] + increment*counter;
	      ++i;
	    }while( vfree[0] < jointMin[freeParams[0]] ||  vfree[0] > jointMax[freeParams[0]] );
	    
	}while (ros::Time::now() < maxTime && vfree[0] >= jointMin[freeParams[0]] &&  vfree[0] <= jointMax[freeParams[0]]);

	error_code = kinematics::NO_IK_SOLUTION; 
 	return false;
   }      
    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles, 
                       std::vector<geometry_msgs::Pose> &poses){
	//ROS_ERROR("getPositionFK"); //Just a mark for entering PositionFK, NO ERROR!!
	KDL::Frame p_out;

	if(link_names.size()!=1 || link_names[0]!=tip_name){
	    ROS_ERROR("Can compute FK for %s only",tip_name.c_str());
	    return false;
	}
	
	bool valid = true;
	
	double eerot[9], eetrans[3];
	fk(&joint_angles[0],eetrans,eerot);
	for(int i=0; i<3;++i) p_out.p.data[i] = eetrans[i];
	for(int i=0; i<9;++i) p_out.M.data[i] = eerot[i];
	
	poses.resize(1);
	tf::PoseKDLToMsg(p_out,poses[0]);	
	
	return valid;
    }      
    std::string getBaseFrame()  {// 	 ROS_ERROR("getBaseFrame");
return root_name; }
    std::string getToolFrame() { //	 ROS_ERROR("getToolFrame");
return tip_name; }
    std::vector<std::string> getJointNames() { //	 ROS_ERROR("getJointNames");
return joints; }
    std::vector<std::string> getLinkNames() { //	 ROS_ERROR("getLinkNames");
return links; }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(cob3_arm_kinematics,IKFastKinematicsPlugin, cob3_arm_kinematics::IKFastKinematicsPlugin, kinematics::KinematicsBase)
