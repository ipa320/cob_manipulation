#include <cob_kinematics/ik_wrapper.h>

#include <ros/assert.h>

#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl_parser/kdl_parser.hpp>

/* FkLookup::ChainFK */

FkLookup::ChainFK::ChainFK(const KDL::Tree& tree, const std::string& root, const std::string& tip):root_name(root),tip_name(tip){
    KDL::Chain chain;
    tree.getChain(root_name, tip_name, chain);
    solver =  new KDL::ChainFkSolverPos_recursive(chain);
    unsigned int  num = chain.getNrOfSegments();
    for(unsigned int i = 0; i < num; ++i){
	const KDL::Joint &joint = chain.getSegment(i).getJoint();
	if (joint.getType() != KDL::Joint::None) joints.insert(std::make_pair(joint.getName(),joints.size()));
    }
    ROS_ASSERT(joints.size() == chain.getNrOfJoints());
}
bool FkLookup::ChainFK::parseJointState(const sensor_msgs::JointState &state_in, KDL::JntArray &array_out, std::map<std::string, unsigned int> &missing) const{
    // seed state > robot_state > planning_scene
   if(missing.size() == 0){
        array_out.resize(joints.size());
        missing = joints;
    }
    int joints_needed = missing.size();
    unsigned int num = state_in.name.size();
    if(state_in.name.size() == state_in.position.size()){
	for(unsigned i = 0; i <  num && joints_needed > 0; ++i){
	    std::map<std::string, unsigned int>::iterator it = missing.find(state_in.name[i]);
	    if( it != missing.end()){
		array_out(it->second) = state_in.position[i];
		--joints_needed;
	    }
	}
    }
    return joints_needed == 0;
}

void FkLookup::ChainFK::getJoints(std::vector<std::string> &v) const{
    for(std::map<std::string, unsigned int>::const_iterator it = joints.begin(); it != joints.end(); ++it)
	v.push_back(it->first);
}

/* FkLookup */

void crawl_and_add_links(const std::vector<boost::shared_ptr<urdf::Link > >  &links, const std::string &root, std::map<std::string, std::string> &target){ // recursion helper
    for(std::vector<boost::shared_ptr<urdf::Link > >::const_iterator it = links.begin(); it!= links.end();++it){
	target.insert(std::make_pair((*it)->name,root));
	crawl_and_add_links((*it)->child_links,root,target); 
    }
}
bool FkLookup::addRoot(const urdf::Model &model, const std::string &root){
    boost::shared_ptr<KDL::Tree> tree;
    tree_map::iterator it = roots.find(root);
    if(it != roots.end()){
	ROS_WARN_STREAM("Link " << root << " already processed");
	return false;
    }
    if(!model.getLink(root)){
	ROS_ERROR_STREAM("Model does not include link '" << root << "'");
	return false;
    }
    tree.reset(new KDL::Tree());
    kdl_parser::treeFromUrdfModel(model, *tree);
    roots.insert(std::make_pair(root,tree));
    
    crawl_and_add_links(model.getLink(root)->child_links,root,tips);
    
    return !tips.empty();
}
const FkLookup::ChainFK* FkLookup::getChain(const std::string &tip){
    if(roots.find(tip) == roots.end()){
	chain_map::iterator it = chains.find(tip);
	if(it != chains.end()){
	    return it->second.get();
	}else{
	    std::map<std::string, std::string>::iterator it_root = tips.find(tip);
	    if(it_root != tips.end()){ // link is valid
		boost::shared_ptr<ChainFK> chain(new ChainFK(*roots[it_root->second],it_root->second,tip));
		chains.insert(std::make_pair(tip, chain));
		return chain.get();
	    }
	}
    }
    return 0;
}

/* IKWrapper */

IKWrapper::IKWrapper(const urdf::Model &model, const std::vector<std::string> root_names,const std::vector<std::string> tip_names){

    for(unsigned int i = 0; i < root_names.size(); ++i){
	std::cout << " " << root_names[i] << std::endl;
	fk_lookup.addRoot(model, root_names[i]);
    }

    for(unsigned int i = 0; i < tip_names.size(); ++i)
	fk_lookup.getChain(tip_names[i]);

}

int IKWrapper::transformPositionIKRequest(kinematics_msgs::PositionIKRequest &request, const geometry_msgs::Pose *pose){
    const FkLookup::ChainFK * fk = fk_lookup.getChain(request.ik_link_name);
    if(fk){
    
	KDL::Frame frame_in, frame_fk, frame_out;
	tf::PoseMsgToKDL(request.pose_stamped.pose, frame_in);

	KDL::JntArray joints;
        std::map<std::string, unsigned int> missing;
	if(!fk->parseJointState(request.ik_seed_state.joint_state, joints, missing)
            && !fk->parseJointState(request.robot_state.joint_state, joints, missing)
            && !fk->parseJointState(robot_state.joint_state, joints, missing) )
        {
            return arm_navigation_msgs::ArmNavigationErrorCodes::INCOMPLETE_ROBOT_STATE;
        }
	
	if(!fk->getFK(joints,frame_fk)) return arm_navigation_msgs::ArmNavigationErrorCodes::NO_FK_SOLUTION;
	
	frame_out = frame_in * frame_fk.Inverse();

	tf::PoseKDLToMsg(frame_out, request.pose_stamped.pose);

	request.ik_link_name = fk->getRoot();
    }
    if(pose){
	KDL::Frame frame_extended, frame_ik, frame_out;
	tf::PoseMsgToKDL(*pose, frame_extended);
	tf::PoseMsgToKDL(request.pose_stamped.pose, frame_ik);
	frame_out = frame_extended.Inverse() * frame_ik;
	tf::PoseKDLToMsg(frame_out, request.pose_stamped.pose);
    }
    return 0;
}
void IKWrapper::getRoots(const std::vector<std::string> &links, std::vector<std::string> &roots_links){
    std::set<std::string> unqiue_roots;
    for(std::vector<std::string>::const_iterator it = links.begin(); it != links.end(); ++it){
	const FkLookup::ChainFK* c = fk_lookup.getChain(*it);    
	if(c){
	    unqiue_roots.insert(c->getRoot());
	}else{
	    unqiue_roots.insert(*it);
	}
    }
    roots_links.assign(unqiue_roots.begin(), unqiue_roots.end());
}
bool IKWrapper::getJoints(const std::vector<std::string> &links, std::vector<std::string> &joints){
    std::set<std::string> unqiue_joints;
    bool res = true;
    for(std::vector<std::string>::const_iterator it = links.begin(); it != links.end(); ++it){
	const FkLookup::ChainFK* c = fk_lookup.getChain(*it);    
	if(c){
	    //unqiue_roots.insert(c->getRoot());
	    //TODO: unqiue_joints
	    std::vector<std::string> j;
	    c->getJoints(j);
	    unqiue_joints.insert(j.begin(),j.end());
	}else{
	    res = false;
	}
    }
    joints.assign(unqiue_joints.begin(), unqiue_joints.end());
    
    return res;
}

bool IKWrapper::getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                     kinematics_msgs::GetPositionFK::Response &response, ros::ServiceClient &get_fk_client){

    std::vector<std::string> &requested_links = response.fk_link_names;
    
    requested_links = request.fk_link_names;
    std::map<std::string, const FkLookup::ChainFK*> chains;
    std::map<std::string, unsigned int> roots_links;
    
    
    for(unsigned int i=0; i < requested_links.size(); ++i){
	const FkLookup::ChainFK* c = fk_lookup.getChain(requested_links[i]);
	chains.insert(std::make_pair(requested_links[i],c));
	if(c){
	    roots_links.insert(std::make_pair(c->getRoot(),0));
	}else{
	    roots_links.insert(std::make_pair(requested_links[i],0));
	}
    }
    
    request.fk_link_names.clear();
    for(std::map<std::string, unsigned int>::iterator it = roots_links.begin(); it != roots_links.end(); ++it){
	it->second= request.fk_link_names.size();
	request.fk_link_names.push_back(it->first);
    }
    
    kinematics_msgs::GetPositionFK::Response response_intern;
    get_fk_client.call(request, response_intern);
    
    response.error_code = response_intern.error_code;
    response.pose_stamped.clear();
    
    if(response_intern.error_code.val ==  arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS){
	for(std::vector<std::string>::iterator it = requested_links.begin(); it != requested_links.end(); ++it){
	    const FkLookup::ChainFK * fk = chains[*it];
	    if(fk == 0){
		response.pose_stamped.push_back(response_intern.pose_stamped[roots_links[*it]]);
		continue;
	    }
	    
	    KDL::Frame frame_fk;

	    KDL::JntArray joints;
            std::map<std::string, unsigned int> missing;
            if(!fk->parseJointState(request.robot_state.joint_state, joints, missing)
                && !fk->parseJointState(robot_state.joint_state, joints, missing) ){
		response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INCOMPLETE_ROBOT_STATE;
		break;
	    }else if(!fk->getFK(joints,frame_fk)){
		response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_FK_SOLUTION;
		break;
	    }else{ // no error
		geometry_msgs::PoseStamped ps = response_intern.pose_stamped[roots_links[fk->getRoot()]];
		KDL::Frame frame_in;
		tf::PoseMsgToKDL(ps.pose, frame_in);

		KDL::Frame frame_out = frame_in * frame_fk;
		
		tf::PoseKDLToMsg(frame_out, ps.pose);
		response.pose_stamped.push_back(ps);
	    }
	}
    }
    if(response.error_code.val !=  arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS){
	response.pose_stamped.clear();
	response.fk_link_names.clear();
    }
    return true;
}
