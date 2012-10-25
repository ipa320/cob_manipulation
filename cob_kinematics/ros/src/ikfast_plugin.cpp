#include <ros/ros.h>
#include <kinematics_base/kinematics_base.h>

namespace IKFAST_NAMESPACE {
 
class IKFastPlugin: public kinematics::KinematicsBase{
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
                                  int &error_code){
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
                                     const double &timeout,
                                     std::vector<double> &solution,
                                     int &error_code){
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
                                     const double &timeout,
                                     std::vector<double> &solution,
                                     const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                     const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                     int &error_code){
                                     }      

      /**
       * @brief Given a set of joint angles and a set of links, compute their pose
       * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
       * @param response - the response contains stamped pose information for all the requested links
       * @return True if a valid solution was found, false otherwise
       */
       virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                  const std::vector<double> &joint_angles, 
                                  std::vector<geometry_msgs::Pose> &poses){
                                  }

      /**
       * @brief  Initialization function for the kinematics
       * @return True if initialization was successful, false otherwise
       */
      virtual bool initialize(std::string name){

      }

      /**
       * @brief  Return the frame in which the kinematics is operating
       * @return the string name of the frame in which the kinematics is operating
       */
      virtual std::string getBaseFrame(){
      }

      /**
       * @brief  Return the links for which kinematics can be computed
       */
      virtual std::string getToolFrame(){
      }

      /**
       * @brief  Return all the joint names in the order they are used internally
       */
      virtual std::vector<std::string> getJointNames(){
      }

      /**
       * @brief  Return all the link names in the order they are represented internally
       */
      virtual std::vector<std::string> getLinkNames(){
      }

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~IKFastPlugin(){}
  };

}
#include <pluginlib/class_list_macros.h>

#define ADD_TYPED_PLUGIN(type,name) PLUGINLIB_DECLARE_CLASS(cob_kinematics, type##name, IKFAST_NAMESPACE::IKFastPlugin, kinematics::KinematicsBase)
#define ADD_IKFAST_PLUGIN(name) ADD_TYPED_PLUGIN(IKFast_, name)
ADD_IKFAST_PLUGIN(IKFAST_NAMESPACE);
  

