#!/bin/bash

robot=$1
pkg_moveit_config_name=${2:-cob_moveit_config}

pkg_moveit_config=`rospack find $pkg_moveit_config_name`

echo "Updating collisions for "$robot" in "$pkg_moveit_config

robot_config=$pkg_moveit_config/robots/$robot/moveit
rosrun moveit_setup_assistant collisions_updater --config-pkg $robot_config --default --always --keep --trials 100000 --xacro-args '--inorder'

echo "Please review changes in "$robot_config" and provide PR!"
