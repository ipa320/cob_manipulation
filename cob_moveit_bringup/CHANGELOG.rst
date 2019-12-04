^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_moveit_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2019-12-04)
------------------
* Merge pull request `#138 <https://github.com/ipa320/cob_manipulation/issues/138>`_ from fmessmer/melodic_checks
  [Melodic] add melodic checks
* fix moveit deprecation warning
* Contributors: Felix Messmer, fmessmer

0.7.2 (2019-05-20)
------------------

0.7.1 (2018-01-07)
------------------
* Merge pull request `#134 <https://github.com/ipa320/cob_manipulation/issues/134>`_ from ipa320/kinetic_release_candidate
  Kinetic release candidate
* Merge pull request `#132 <https://github.com/ipa320/cob_manipulation/issues/132>`_ from ipa-fxm/kinetic_dev
  [kinetic] updates from indigo_dev
* Merge branch 'indigo_dev' of github.com:ipa320/cob_manipulation into kinetic_dev
  Conflicts:
  .travis.yml
  cob_kinematics/ikfast/src/ikfast_lbr.cpp
  cob_kinematics/ikfast/src/ikfast_ur10.cpp
  cob_kinematics/ikfast/src/ikfast_ur5.cpp
  cob_kinematics/package.xml
  cob_kinematics/ros/bin/genikfast.py
  cob_kinematics/ros/src/ikfast_plugin.cpp
  cob_kinematics/ros/src/urdf_openrave.cpp
* Merge pull request `#128 <https://github.com/ipa320/cob_manipulation/issues/128>`_ from ipa-fxm/configurable_moveit_config_helper
  additional arguments for moveit_config helper
* simplify template substitution
* fix setup assistant jade xacro support
* additional arguments for moveit_config helper
* Merge pull request `#124 <https://github.com/ipa320/cob_manipulation/issues/124>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#120 <https://github.com/ipa320/cob_manipulation/issues/120>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* Merge pull request `#123 <https://github.com/ipa320/cob_manipulation/issues/123>`_ from ipa-fxm/fix_launch_arguments
  properly pass missing launch arguments
* properly pass missing launch arguments
* updated author/maintainer
* use license apache 2.0
* Contributors: Felix Messmer, Mathias Lüdtke, Richard Bormann, ipa-fxm, ipa-uhr-mk

0.7.0 (2017-07-31)
------------------
* remove stomp configuration
  Conflicts:
  .travis.rosinstall
* Contributors: ipa-fxm

0.6.5 (2017-07-31)
------------------
* remove stomp configuration
* Revert "Adding chomp planning pipeline"
* chomp planning pipeline file added robot argument
* restructure moveit config
* Update planning_context.xml
* Stomp planner (`#104 <https://github.com/ipa320/cob_manipulation/issues/104>`_)
  * moveit setup with UR arm
  * Arm moveit configuration
  * Controller corrected
  * Arm with gripper moveit config
  * solver options tunned
  * moveit whole body configuration planning for raw3-1
  * whole body configuration with virtual joint working
  * Stomp configuration files created for rob@work arm
  * stomp configuration for raw3-1 created and tested
  * changes from pull request
  * new change from pull request
  * added travis rosinstall dependencies for industrial_moveit
  * identation fixed in travis.rosinstall
  * NEW FIX TO INDENTATION
  * proper rosdep key for occupancy grid  monitor
  * rosinstall chnaged to my local fork to test install tags
  * stomp moveit link change to original github browser after pull request being accepted
  * version updated
  * pull request changes
* add xacro-args --inorder
* fix moveit deprecation warning
* scripts for updating collisions in moveit_config
* move setup_assistant launch file
* do not load robot_description in cob_moveit_config/upload_config.launch
* minor launch adjustments
* fix database location
* split, move and adjust launch files
* new package cob_moveit_bringup
* Contributors: Bruno Brito, Felix Messmer, Mayank_Patel, ipa-fxm
