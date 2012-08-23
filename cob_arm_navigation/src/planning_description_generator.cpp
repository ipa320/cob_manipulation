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

template <class T> void sort_pairs(const T &input, vector<CollisionOperationsGenerator::StringPair> &target){
	set<CollisionOperationsGenerator::StringPair> vset;
	for(typename T::const_iterator it = input.begin();it != input.end(); ++it){
		if (it->first < it->second){
			vset.insert(*it);
		}else{
			vset.insert(make_pair(it->second, it->first));
		}
	}
	target.assign(vset.begin(),vset.end());
}

void emit_config_yaml(YAML::Emitter *emitter_, const vector<KinematicModel::MultiDofConfig> &multi_dof_configs, const map<string, KinematicModel::GroupConfig> &group_config_map){
	(*emitter_) << YAML::BeginMap;

	(*emitter_) << YAML::Key << "multi_dof_joints";
	(*emitter_) << YAML::Value << YAML::BeginSeq;
	for(vector<KinematicModel::MultiDofConfig>::const_iterator it = multi_dof_configs.begin();  it != multi_dof_configs.end(); ++it){
		(*emitter_) << YAML::BeginMap;
		(*emitter_) << YAML::Key << "name" << YAML::Value << it->name;
		(*emitter_) << YAML::Key << "type" << YAML::Value << it->type;
		(*emitter_) << YAML::Key << "parent_frame_id" << YAML::Value << it->parent_frame_id;
		(*emitter_) << YAML::Key << "child_frame_id" << YAML::Value << it->child_frame_id;
		(*emitter_) << YAML::EndMap;
	}
	(*emitter_) << YAML::EndSeq;

  (*emitter_) << YAML::Key << "groups";
  (*emitter_) << YAML::Value << YAML::BeginSeq;


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

  (*emitter_) << YAML::EndMap;
}

int main(int argc, char **argv){
    
    srand(time(NULL));
    ros::init(argc, argv, "planning_description_generator",ros::init_options::AnonymousName);

    boost::shared_ptr<urdf::Model> urdf_(new urdf::Model);
    planning_models::KinematicModel* kmodel_ = 0;
    planning_environment::CollisionModels* cm_ = 0;
    planning_environment::CollisionOperationsGenerator* ops_gen_ = 0;
    collision_space::EnvironmentModel* ode_collision_model_ = 0;
    vector<KinematicModel::MultiDofConfig> multi_dof_configs;
    map<CollisionOperationsGenerator::DisableType,
	   vector<CollisionOperationsGenerator::StringPair> > disable_map_;

    set<CollisionOperationsGenerator::StringPair> already_disabled;

    // parse params

    string file_urdf, file_config, file_collisions, type;
    bool write_config = false;
    bool shrink_mode = false;

    map<string,pair<string,string> > arms;

    {	int i = 1;
    	while(i < argc){
			if(strcmp(argv[i],"--urdf") == 0)
				file_urdf = argv[++i];
			else if(strcmp(argv[i],"--output") == 0)
				file_collisions = argv[++i];
			else if(strcmp(argv[i],"--config") == 0)
				file_config = argv[++i];
			else if(strcmp(argv[i],"--safety") == 0)
				type = argv[++i];
			else if(strcmp(argv[i],"--shrink") == 0)
				shrink_mode = true;
			else{
				write_config = true;
				arms.insert(make_pair(string(argv[i]),make_pair(string(argv[i+1]),string(argv[i+2]))));
				i+=2;
			}
			++i;
		}
        if(i > argc){
        	ROS_ERROR("planning_description_generator [--urdf PATH] [--config PATH] [--output PATH] [--shrink] [--safety TYPE] [ARM_NAME ARM_ROOT ARM_TIP] [...]");
        	return -1;
        }
    }

    // read arm from output file if possible
    if(!file_config.empty()){
        std::ifstream fin(file_config.c_str());
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
				const YAML::Node &mdj = doc["multi_dof_joints"];
				for(YAML::Iterator it=mdj.begin();it!=mdj.end();++it) {
					try {
						string name;
						(*it)["name"] >>  name;
				        planning_models::KinematicModel::MultiDofConfig joint_config(name);
						(*it)["type"] >>  joint_config.type;
						(*it)["parent_frame_id"] >>  joint_config.parent_frame_id;
						(*it)["child_frame_id"] >>  joint_config.child_frame_id;
					    multi_dof_configs.push_back(joint_config);
					}catch(YAML::Exception &e) {
						continue;
					}
                }
            }
        }catch(YAML::Exception &e) { write_config = true;}
    }
    
    // read all disabled pairs in shrink mode
    if(!file_collisions.empty() && shrink_mode){
        std::ifstream fin(file_collisions.c_str());
        try {
            YAML::Parser parser(fin);
            YAML::Node doc;
            if(parser.GetNextDocument(doc)) {
				const YAML::Node &ops = doc["default_collision_operations"];
				for(YAML::Iterator it=ops.begin();it!=ops.end();++it) {
					try {
						string op, o1, o2;
						(*it)["operation"] >>  op;
						(*it)["object1"] >>  o1;
						(*it)["object2"] >>  o2;
						if(op=="disable"){
							if(o1 > o2)  already_disabled.insert( make_pair(o2,o1) );
							else already_disabled.insert( make_pair(o1,o2) );
						}
					}catch(YAML::ParserException& e) {
						continue;
					}
				}
            }
        }catch(YAML::Exception &e) { }
    }
    if(arms.empty()){
        ROS_ERROR("No kinematic chains specified!");
        return -1;
    }
    
    // load model

    string xml;

    if(file_urdf.empty()){
		stringstream sstream;
		string line;
		while( getline(cin, line)) {
			sstream << line << endl;
		}
		xml = sstream.str();
    } else xml = exec((string("rosrun xacro xacro.py ")+file_urdf).c_str());
    
    if(!urdf_->initString(xml))
    {
		ROS_ERROR_STREAM("Parsing URDF failed!");
		return -1;
    }
      
    vector<KinematicModel::GroupConfig> gcs;
    const urdf::Link *root = urdf_->getRoot().get();

    if(multi_dof_configs.empty()){
        planning_models::KinematicModel::MultiDofConfig world_joint_config_("world_joint");
		world_joint_config_.type = "Floating";
		world_joint_config_.parent_frame_id = "odom_combined";
		world_joint_config_.child_frame_id = root->name;
	    multi_dof_configs.push_back(world_joint_config_);
	    write_config = true;
    }

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
	map<string, bool> cdof_map;
    //assuming that 0th is world joint, which we don't want to include
    for(unsigned int i = 1; i < jmv.size(); i++)
    {
		const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
		for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
		{
			cdof_map[it->first] = true;
		}
	}
    ops_gen_->generateSamplingStructures(cdof_map);


    // generate collision operations
    
    ROS_INFO("Starting..");

    CollisionOperationsGenerator::SamplingSafety safety;
    if(type.empty()) safety = CollisionOperationsGenerator::Normal;
    else if(type == "VerySafe") safety = CollisionOperationsGenerator::VerySafe;
    else if(type == "Safe") safety =  CollisionOperationsGenerator::Safe;
    else if(type == "Normal") safety = CollisionOperationsGenerator::Normal;
    else if(type == "Fast") safety = CollisionOperationsGenerator::Fast;
    else if(type == "VeryFast") safety = CollisionOperationsGenerator::VeryFast;
    else{
    	ROS_ERROR("Supported types are (case sensitive): VerySafe Safe Normal(=default) Fast VeryFast");
    	return -1;
    }

    ops_gen_->setSafety(safety);

    vector<CollisionOperationsGenerator::StringPair> adj_links; 
    ops_gen_->generateAdjacentInCollisionPairs(adj_links);
    ops_gen_->disablePairCollisionChecking(adj_links);
    //disable_map_[CollisionOperationsGenerator::ADJACENT] = adj_links;
    sort_pairs(adj_links, disable_map_[CollisionOperationsGenerator::ADJACENT]);

    vector<CollisionOperationsGenerator::StringPair> always_in_collision;
    vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;
    ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);
    
    ops_gen_->disablePairCollisionChecking(always_in_collision);
    //disable_map_[CollisionOperationsGenerator::ALWAYS] = always_in_collision;
    sort_pairs(always_in_collision, disable_map_[CollisionOperationsGenerator::ALWAYS]);
  
    vector<CollisionOperationsGenerator::StringPair> default_in_collision;
    ops_gen_->generateDefaultInCollisionPairs(default_in_collision, in_collision_joint_values);
    ops_gen_->disablePairCollisionChecking(default_in_collision);
    //disable_map_[CollisionOperationsGenerator::DEFAULT] = default_in_collision;
    sort_pairs(default_in_collision, disable_map_[CollisionOperationsGenerator::DEFAULT]);
    
    vector<CollisionOperationsGenerator::StringPair> often_in_collision;
    vector<double> percentages;
    ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);
    ops_gen_->disablePairCollisionChecking(often_in_collision);
    //disable_map_[CollisionOperationsGenerator::OFTEN] = often_in_collision;
    sort_pairs(often_in_collision, disable_map_[CollisionOperationsGenerator::OFTEN]);
    
    vector<CollisionOperationsGenerator::StringPair> not_in_collision;
    vector<CollisionOperationsGenerator::StringPair> fast_collisions;

    ops_gen_->setSafety(CollisionOperationsGenerator::Fast);
    ops_gen_->generateOccasionallyAndNeverInCollisionPairs(fast_collisions, not_in_collision, percentages,
							 in_collision_joint_values);
    ops_gen_->disablePairCollisionChecking(fast_collisions);
    ops_gen_->setSafety(safety);

    vector<CollisionOperationsGenerator::StringPair> in_collision;
    ops_gen_->generateOccasionallyAndNeverInCollisionPairs(in_collision, not_in_collision, percentages,
							 in_collision_joint_values);
    //disable_map_[CollisionOperationsGenerator::NEVER] = not_in_collision;
    
	list<CollisionOperationsGenerator::StringPair> never(not_in_collision.begin(),not_in_collision.end());
	for(std::vector<CollisionOperationsGenerator::StringPair>::iterator it = fast_collisions.begin(); it != fast_collisions.end(); ++it)
	{
		never.remove(*it);
		never.remove(make_pair(it->second, it->first));
	}
	for(std::list<CollisionOperationsGenerator::StringPair>::iterator it = never.begin(); it != never.end();)
	{
		if(already_disabled.find(*it)== already_disabled.end() && already_disabled.find(make_pair(it->second, it->first)) == already_disabled.end()){
			it = never.erase(it);
		}else{
			++it;
		}
	}
	//disable_map_[CollisionOperationsGenerator::NEVER].assign(never.begin(),never.end());
    sort_pairs(never, disable_map_[CollisionOperationsGenerator::NEVER]);


    // output planning config
    if(write_config){
		YAML::Emitter *emitter_ = new YAML::Emitter;
		const map<string, KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();
		emit_config_yaml(emitter_, multi_dof_configs, group_config_map);
		if(file_config.empty())
			cout << emitter_->c_str();
		else{
			ofstream of(file_config.c_str());
			of <<  emitter_->c_str();
		}
	  delete emitter_;
    }

  //ops_gen_->performanceTestSavedResults(disable_map_);
	YAML::Emitter *emitter_ = new YAML::Emitter;
	(*emitter_) << YAML::BeginMap;
    ops_gen_->outputYamlStringOfSavedResults((*emitter_), disable_map_);
    //end map
    (*emitter_) << YAML::EndMap;

    if(file_collisions.empty())
    	cout << emitter_->c_str();
    else{
    	ofstream of(file_collisions.c_str());
    	of <<  emitter_->c_str();
    }
    delete emitter_;
    
    return 0;
}

