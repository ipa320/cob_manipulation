#ifndef I_IK_WRAPPER_H
#define I_IK_WRAPPER_H

#include <ros/ros.h>

#include <urdf/model.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/RobotState.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/tree.hpp>

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

class FkLookup{
public:    
    class ChainFK{
	const std::string root_name;
	const std::string tip_name;
	std::map<std::string, unsigned int> joints;
	KDL::ChainFkSolverPos *solver;
    public:
        ChainFK():solver(0){}
        ChainFK(const KDL::Tree &tree, const std::string &root, const std::string &tip);
        virtual ~ChainFK(){ if (solver) { delete solver; solver = 0; } }
	bool parseJointState(const sensor_msgs::JointState &state_in, KDL::JntArray &array_out, std::map<std::string, unsigned int> &missing) const;
	bool getFK(const KDL::JntArray &joints, KDL::Frame & frame_out) const { return solver != 0 && solver->JntToCart(joints,frame_out) == 0; }
	const std::string& getTip() const { return tip_name; }
	const std::string& getRoot() const { return root_name; }
	void getJoints(std::vector<std::string> &v) const;
    };
    bool addRoot(const urdf::Model &model, const std::string &root);
    const ChainFK * getChain(const std::string &tip);
private:    
    typedef std::map<std::string, boost::shared_ptr<ChainFK> > chain_map;
    typedef std::map<std::string, boost::shared_ptr<KDL::Tree> > tree_map;
    chain_map chains;
    tree_map roots;
    std::map<std::string, std::string> tips;
};

class IKWrapper{   
    FkLookup fk_lookup;
    arm_navigation_msgs::RobotState robot_state;
    
public:
    IKWrapper(const urdf::Model &model, const std::vector<std::string> root_names,const std::vector<std::string> tip_names);

    void updateRobotState(arm_navigation_msgs::RobotState &rs){
        robot_state = rs; // no lock -> lock it externally!
    }

    int transformPositionIKRequest(kinematics_msgs::PositionIKRequest &request, const geometry_msgs::Pose *pose = 0);
    void getRoots(const std::vector<std::string> &links, std::vector<std::string> &roots_links);
    bool getJoints(const std::vector<std::string> &links, std::vector<std::string> &joints);
		     
    bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                     kinematics_msgs::GetPositionFK::Response &response, ros::ServiceClient &get_fk_client);
};
#endif /* !I_IK_WRAPPER_H */
