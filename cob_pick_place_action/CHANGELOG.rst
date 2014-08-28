^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_pick_place_action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* python indentation fix
* Contributors: ipa-fxm

0.5.1 (2014-03-26)
------------------
* use BIN_DESTINATION
* beautify install tags and correct PYTHON_DESTINATION
* install tags
* catkin_lint and install tags
* add changelogs
* Merge pull request `#11 <https://github.com/ipa320/cob_manipulation/issues/11>`_ from ipa-fxm/groovy_dev
  bring groovy updates to hydro
* use object_class to determine mesh_file in insertObject
* backup from cob3-3
* move cob_mmcontroller + groovy_updates
* pick_place now works with released version of moveit
* next try
* catkinized + fixes for changed message types
* changes in action definition + according changes in code
* multi-place upright example
* changes for multi-pick-multi-place + example
* changes for multi-pick-multi-place + example
* test place
* fixes after merge
* minor
* adapt to new action in grasp_generation
* delete obsolete files
* approach and retreat is now wrt arm_7_link
* use random in pick_client
* additional params in action (threshold,grasp_id,num_grasps) + adaptions + some improvements
* backup
* introducing ALL grasps, some debug output
* merge with ws
* show output only in debug mode
* latest r3cop status
* testing
* better approach retreat
* object position changed
* Merge branch 'pick_n_place' of https://github.com/ipa-fmw-ws/cob_manipulation into pick_n_place
* Merge remote-tracking branch 'origin-fxm/pick_n_place' into pick_n_place
* small changes in parameters
* grasp hotpot
* fixed grasp_generation action client
* added launch file for pick-n-place with grasp_generation
* Merge branch 'pick_n_place' of https://github.com/ipa-fmw-ws/cob_manipulation into pick_n_place
* db hotpot added, new preshapes
* Merge branch 'pick_n_place' of https://github.com/ipa-fmw-ws/cob_manipulation into pick_n_place
* first try with openrave
* integration of openrave
* integration of openrave
* better approach and retreat strategy
* all meshes have translation, rotation and scale applied + new objects for r3cop
* remove all setupEnvironment stuff from pick_place
* mesh with applied translation, rotation and scale
* fixed yellowsaltcube
* fixed rotation of meshes
* Merge branch 'r3_cop_pick-n-place' of github.com:ipa-fxm/cob_manipulation into pick_n_place
* fixed lookupTransform problems with tf_listener
* integration of OpenRAVE-grasp-generation + beautifying
* added missing action description
* better retreat + beautifying
* transform sdh_palm-arm7 added - still problems with lookupTransform
* fixed mergeconficts
* merge
* using cob_moveit_interface within pick-n-place
* merge
* added destination pose
* added many more objects
* pick and place python client
* use last grasp from pick within place
* implementation of place action and transform pose
* fixing insertObject using meshes
* adding new grasptables
* cleaning up
* added grasp_id field to action description
* adding meshes and grasptables for new objects
* fixing meshes scale
* introducing relative paths for grasptables - modifications in setup environment
* relative path for .xml's
* fix typo, increase planning time, use fillAllGrasps()
* RELATIVE PATH TO GRASPTABLE(XML)
* calculate approach direction wrt footprint
* find GraspTable.txt in package instead of hardcoded absolute path
* beautifying and minor improvements
* minor changes
* minor changes
* added class GraspTable for parsing GraspTables from KIT database
* adding mesh for objects from KIT database
* adding GraspTables from KIT database
* added pick action_client
* added pick action_server
* define action
* initial commit of cob_pick_place_action
* Contributors: Florian Weisshardt, Jan Fischer, Witalij Siebert, ipa-fxm, rohit chandra
