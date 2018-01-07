#!/bin/bash

robot=$1
pkg_moveit_config_name=${2:-cob_moveit_config}
pkg_hardware_config_name=${3:-cob_hardware_config}

pkg_moveit_config=`rospack find $pkg_moveit_config_name`

echo "Adding moveit_config for "$robot" in "$pkg_moveit_config

mkdir -p $pkg_moveit_config/robots/$robot/moveit/config

cat <<SRDF_TEMPLATE > $pkg_moveit_config/robots/$robot/moveit/config/$robot.srdf
<robot name="$robot">
</robot>
SRDF_TEMPLATE

cat <<CONFIG_TEMPLATE > $pkg_moveit_config/robots/$robot/moveit/.setup_assistant
moveit_setup_assistant_config:
  URDF:
    package: $pkg_hardware_config_name
    relative_path: robots/$robot/urdf/$robot.urdf.xacro
    use_jade_xacro: true
  SRDF:
    relative_path: config/$robot.srdf
CONFIG_TEMPLATE
