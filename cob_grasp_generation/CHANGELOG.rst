^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_grasp_generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.5.1 (2014-03-26)
------------------
* Merge branch 'hydro_dev' into hydro_release_candidate
* use BIN_DESTINATION
* beautify install tags and correct PYTHON_DESTINATION
* update package maintainer
* Merge branch 'hydro_dev' into hydro_release_candidate
* install tags
* catkin_lint and install tags
* add changelogs
* move cob_mmcontroller + groovy_updates
* fix deps
* pick_place now works with released version of moveit
* fixes for changed message types
* catkinized
* manually remove spreizgriffe
* removed bottom grasps from grasp tables
* generating new grasps
* better grasptable for instanttomatosoup
* better grasptable for hotpot2
* better grasptable for hotpot
* fixes after merge
* adaptions, fixes and new generation_strategy
* cleaning up
* restructuring folders
* new action_clients
* combined action_server
* new actions
* delete obsolete files
* new grasp_tables
* minor fixes
* additional params in action (threshold,grasp_id,num_grasps) + adaptions + some improvements
* close fingers a little more so that objects dont slip through
* fully implemented as class + improvements
* merge with ws
* Merge branch 'pick_n_place' of https://github.com/ipa-fmw-ws/cob_manipulation into pick_n_place
* remove obsolete sleep
* server now uses class from or_grasp_generation and a threshold
* added threshold, num_grasps and grasp_id for grasp database
* start implementation as class
* remove unused parameter
* improved show grasp
* removed some unfeasible grasps manually
* Removed Salt textures
* Added service server for showing grasps
* showgrasp functionality added
* sort call fixed, hardcoded paths fixed
* fixed sorting algorithm
* added hotpo2 to DB
* added hotpot to DB
* new DB for salt and tomatosoup
* pre_joint_config changed
* removed wrong DBs
* preshapes set to cylindric only
* fixing negative zero values
* Merge branch 'pick_n_place' of github.com:ipa-fxm/cob_manipulation into pick_n_place
* working on grasp view
* added grasptable for instanttomatosoup
* find package_paths using roslib
* Merge branch 'pick_n_place' of https://github.com/ipa-fmw-ws/cob_manipulation into pick_n_place
* add new db fruittea
* Todo added
* Merge remote-tracking branch 'origin-fxm/pick_n_place' into pick_n_place
* new structure
* objects now created dynamically from a mesh
* objects removed from scene
* fixed output for action server
* fixed naming of grasp_generation action
* removed tmp files
* added hotpot2
* new object hotpot and hotpot2
* saltcube with new preshapes
* db hotpot added, new preshapes
* first try with openrave
* first database for new meshes
* small changes
* kinbody for new meshfiles without scale
* modified to work with the new mesh files now
* database generated for transformed mesh file
* objects are now taken from cob_pick_place_action
* check if db exists for specific object before start planning with openrave
* output now a grasp list
* hardcoded paths fixed
* latest commit
* created with service call
* client can use object_id now for service calls
* moved to src
* service server call is working now
* salt xml for openrave
* salt mesh for testing
* added scene
* runs independently now
* clean up code
* README file added
* changed serice files from src to scripts
* new package for grasp_generation action
* Contributors: Florian Weisshardt, Jan Fischer, Witalij Siebert, ipa-fxm
