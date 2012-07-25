#include <ros/ros.h>

#include <planning_environment/models/collision_models.h>
#include <collision_space/environmentODE.h>
#include <planning_environment/util/collision_operations_generator.h>
#include <algorithm>
#include <yaml-cpp/yaml.h>


using namespace std;
using namespace planning_environment;
using namespace planning_models;
using namespace collision_space;

std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
                result += buffer;
    }
    pclose(pipe);
    return result;
}

int main(int argc, char **argv){
    
    srand(time(NULL));
    ros::init(argc, argv, "planning_description_generator",ros::init_options::AnonymousName);

    boost::shared_ptr<urdf::Model> urdf_(new urdf::Model);
    planning_models::KinematicModel* kmodel_ = 0;
    planning_environment::CollisionModels* cm_ = 0;
    planning_environment::CollisionOperationsGenerator* ops_gen_ = 0;
    collision_space::EnvironmentModel* ode_collision_model_ = 0;
    planning_models::KinematicModel::MultiDofConfig world_joint_config_("world_joint");
    std::map<planning_environment::CollisionOperationsGenerator::DisableType, 
	   std::vector<planning_environment::CollisionOperationsGenerator::StringPair> > disable_map_;

    // parse params

    string file_in, file_out, type;

    map<string,pair<string,string> > arms;

    {	int i = 1;
    	while(i < argc){
			if(strcmp(argv[i],"--input") == 0)
				file_in = argv[++i];
			else if(strcmp(argv[i],"--output") == 0)
				file_out = argv[++i];
			else if(strcmp(argv[i],"--type") == 0)
				type = argv[++i];
			else{
				arms.insert(make_pair(string(argv[i]),make_pair(string(argv[i+1]),string(argv[i+2]))));
				i+=2;
			}
			++i;
		}
        if(i > argc){
        	ROS_ERROR("planning_description_generator [--input PATH] [--output PATH] [--type TYPE] [ARM_NAME ARM_ROOT ARM_TIP] [...]");
        	return -1;
        }
    }

    // read arm from output file if possible
    if(!file_out.empty()){
        std::ifstream fin(file_out.c_str());
        try {
            YAML::Parser parser(fin);
            YAML::Node doc;
            if(parser.GetNextDocument(doc)) {
                const YAML::Node &groups = doc["groups"];
                 for(YAML::Iterator it=groups.begin();it!=groups.end();++it) {
                    try {
                        string group, tip, root;
                        (*it)["name"] >>  group;
                        (*it)["tip_link"] >>  tip;
                        (*it)["base_link"] >>  root;
                         arms.insert(make_pair(group,make_pair(root,tip)));
                    }catch(YAML::ParserException& e) {
                        continue;
                    }            
                }
            }
        }catch(YAML::ParserException& e) { /* nothing do to */ }            
    }
    
    if(arms.empty()){
        ROS_ERROR("No kinematic chains specified!");
        return -1;
    }
    
    // load model

    string xml;

    if(file_in.empty()){
		stringstream sstream;
		string line;
		while( getline(cin, line)) {
			sstream << line << endl;
		}
		xml = sstream.str();
    } else xml = exec((string("rosrun xacro xacro.py ")+file_in).c_str());
    
    if(!urdf_->initString(xml))
    {
		ROS_ERROR_STREAM("Parsing URDF failed!");
		return -1;
    }
      
    vector<KinematicModel::GroupConfig> gcs;
    vector<KinematicModel::MultiDofConfig> multi_dof_configs;
    const urdf::Link *root = urdf_->getRoot().get();

    //now this should work with an n=on-identity transform
    world_joint_config_.type = "Floating";
    world_joint_config_.parent_frame_id = "odom_combined";
    world_joint_config_.child_frame_id = root->name;
    multi_dof_configs.push_back(world_joint_config_);

    kmodel_ = new KinematicModel(*urdf_, gcs, multi_dof_configs);

    ode_collision_model_ = new EnvironmentModelODE();

    const vector<KinematicModel::LinkModel*>& coll_links = kmodel_->getLinkModelsWithCollisionGeometry();

    vector<string> coll_names;
    for(unsigned int i = 0; i < coll_links.size(); i++)
    {
	coll_names.push_back(coll_links[i]->getName());
    }
    EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names, false);
    map<string, double> default_link_padding_map;
    ode_collision_model_->setRobotModel(kmodel_, default_collision_matrix, default_link_padding_map, 0.0, 1.0);

    cm_ = new CollisionModels(urdf_, kmodel_, ode_collision_model_);
    ops_gen_ = new CollisionOperationsGenerator(cm_);
    
    // get arms
    
    for(map<string,pair<string,string> >::iterator it=arms.begin(); it != arms.end(); ++it){
                ROS_INFO_STREAM("Chain: " << it->first);
		KinematicModel::GroupConfig arm_gc(it->first, it->second.first, it->second.second);
		kmodel_->addModelGroup(arm_gc);
    }

    // disable intra robot collision checking

//    vector<string> non_arm_links;
//    kmodel_->getLinkModelNames(non_arm_links);
//    list<string> non_arm_links_list(non_arm_links.begin(),non_arm_links.end());
//    const vector<string> &updated_links = kmodel_->getModelGroup("arm")->getUpdatedLinkModelNames();
//    std::set< std::string >  arm_links_set(updated_links.begin(),updated_links.end());
//
//    for(list<string>::iterator it = non_arm_links_list.begin(); it != non_arm_links_list.end(); ++it)
//    	if(arm_links_set.find(*it) != arm_links_set.end())
//    		it = non_arm_links_list.erase(it);
//
//
//    for(list<string>::iterator it1 = non_arm_links_list.begin(); it1 != non_arm_links_list.end(); ++it1)
//    	for(list<string>::iterator it2 = non_arm_links_list.begin(); it2 != non_arm_links_list.end(); ++it2)
//    		ops_gen_->disablePairCollisionChecking(make_pair(*it1,*it2));

    // set DOF
    const vector<KinematicModel::JointModel*>& jmv = kmodel_->getJointModels();
    vector<bool> consider_dof;
    //assuming that 0th is world joint, which we don't want to include
    for(unsigned int i = 1; i < jmv.size(); i++)
    {
		const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
		for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
		{
		  consider_dof.push_back(true);
		}
	}
	int xind = 0;
	map<string, bool> cdof_map;
	for(unsigned int i = 1; i < jmv.size(); i++)
	{
		const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
		for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
		{
		  cdof_map[it->first] = consider_dof[xind++];
                  ROS_INFO_STREAM(it->first << " " << it->second.first << " " << it->second.second);
		}
    }
    ops_gen_->generateSamplingStructures(cdof_map);


    // generate collision operations
    
    ROS_INFO("Starting..");

    if(type.empty()) ops_gen_->setSafety(CollisionOperationsGenerator::Normal);
    else if(type == "VerySafe") ops_gen_->setSafety(CollisionOperationsGenerator::VerySafe);
    else if(type == "Safe") ops_gen_->setSafety(CollisionOperationsGenerator::Safe);
    else if(type == "Normal") ops_gen_->setSafety(CollisionOperationsGenerator::Normal);
    else if(type == "Fast") ops_gen_->setSafety(CollisionOperationsGenerator::Fast);
    else if(type == "VeryFast") ops_gen_->setSafety(CollisionOperationsGenerator::VeryFast);
    else{
    	ROS_ERROR("Supported types are (case sensitive): VerySafe Safe Normal(=default) Fast VeryFast");
    	return -1;
    }

    vector<CollisionOperationsGenerator::StringPair> adj_links; 
    ops_gen_->generateAdjacentInCollisionPairs(adj_links);
    ops_gen_->disablePairCollisionChecking(adj_links);
    disable_map_[CollisionOperationsGenerator::ADJACENT] = adj_links;

    vector<CollisionOperationsGenerator::StringPair> always_in_collision;
    vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;
    ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);
    
    ops_gen_->disablePairCollisionChecking(always_in_collision);
    disable_map_[CollisionOperationsGenerator::ALWAYS] = always_in_collision;
  
    vector<CollisionOperationsGenerator::StringPair> default_in_collision;
    ops_gen_->generateDefaultInCollisionPairs(default_in_collision, in_collision_joint_values);
    ops_gen_->disablePairCollisionChecking(default_in_collision);
    disable_map_[CollisionOperationsGenerator::DEFAULT] = default_in_collision;
    
    vector<CollisionOperationsGenerator::StringPair> often_in_collision;
    vector<double> percentages;
    ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);
    ops_gen_->disablePairCollisionChecking(often_in_collision);
    disable_map_[CollisionOperationsGenerator::OFTEN] = often_in_collision;
    
    vector<CollisionOperationsGenerator::StringPair> in_collision;
    vector<CollisionOperationsGenerator::StringPair> not_in_collision;

    ops_gen_->generateOccasionallyAndNeverInCollisionPairs(in_collision, not_in_collision, percentages,
							 in_collision_joint_values);
    disable_map_[CollisionOperationsGenerator::NEVER] = not_in_collision;
    ops_gen_->disablePairCollisionChecking(not_in_collision);
    
    // output planning description
    YAML::Emitter *emitter_ = new YAML::Emitter;
    (*emitter_) << YAML::BeginMap;

    (*emitter_) << YAML::Key << "multi_dof_joints";
    (*emitter_) << YAML::Value << YAML::BeginSeq;
    (*emitter_) << YAML::BeginMap;
    (*emitter_) << YAML::Key << "name" << YAML::Value << world_joint_config_.name;
    (*emitter_) << YAML::Key << "type" << YAML::Value << world_joint_config_.type;
    (*emitter_) << YAML::Key << "parent_frame_id" << YAML::Value << world_joint_config_.parent_frame_id;
    (*emitter_) << YAML::Key << "child_frame_id" << YAML::Value << world_joint_config_.child_frame_id;
    (*emitter_) << YAML::EndMap;
    (*emitter_) << YAML::EndSeq;

  (*emitter_) << YAML::Key << "groups";
  (*emitter_) << YAML::Value << YAML::BeginSeq;

  const map<string, KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

  for(map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it
        != group_config_map.end(); it++)
  {
    (*emitter_) << YAML::BeginMap;
    (*emitter_) << YAML::Key << "name" << YAML::Value << it->first;
    if(!it->second.base_link_.empty())
    {
      (*emitter_) << YAML::Key << "base_link" << YAML::Value << it->second.base_link_;
      (*emitter_) << YAML::Key << "tip_link" << YAML::Value << it->second.tip_link_;
    }
    else
    {
      if(!it->second.subgroups_.empty())
      {
        (*emitter_) << YAML::Key << "subgroups";
        (*emitter_) << YAML::Value << YAML::BeginSeq;
        for(unsigned int i = 0; i < it->second.subgroups_.size(); i++)
        {
          (*emitter_) << it->second.subgroups_[i];
        }
        (*emitter_) << YAML::EndSeq;
      }
      if(!it->second.joints_.empty())
      {
        (*emitter_) << YAML::Key << "joints";
        (*emitter_) << YAML::Value << YAML::BeginSeq;
        for(unsigned int i = 0; i < it->second.joints_.size(); i++)
        {
          (*emitter_) << it->second.joints_[i];
        }
        (*emitter_) << YAML::EndSeq;
      }
    }
    (*emitter_) << YAML::EndMap;
  }
  (*emitter_) << YAML::EndSeq;


    //ops_gen_->performanceTestSavedResults(disable_map_);
    ops_gen_->outputYamlStringOfSavedResults((*emitter_), disable_map_);
    //end map
    (*emitter_) << YAML::EndMap;

    if(file_out.empty())
    	cout << emitter_->c_str();
    else{
    	ofstream of(file_out.c_str());
    	of <<  emitter_->c_str();
    }
    
    return 0;
}

