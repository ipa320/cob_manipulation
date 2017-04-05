#!/bin/bash

robot=$1

moveit_config=`rospack find cob_moveit_config`
config_pkg=$moveit_config/$robot

echo "Updating collisions in "$config_pkg

rosrun moveit_setup_assistant collisions_updater --config-pkg $config_pkg --default --always --keep --trials 100000 --xacro-args '--inorder'

echo "Please review changes in "$config_pkg" and provide PR!"
