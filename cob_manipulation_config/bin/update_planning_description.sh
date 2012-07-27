#!/bin/bash
if [ $# -lt 1 ]
then
    echo "Usage: update_planning_description ROBOT"
    echo "It may take hours to finish!"
    exit 0
elif [ "$1" == "ALL" ]
then
    robots=""
    for d in `rospack find cob_manipulation_config`/*/default_collision_operations.yaml; do robots="$robots $(basename $(dirname $d))"; done    
else
    robots=$1
fi
if [ $# -eq 2 ]
then
    safety=$2
else
    safety=VerySafe
fi

trap "kill 0" SIGINT SIGTERM EXIT
roscore&
disown $!
for r in $robots
do
    echo $r
    config=$(rospack find cob_manipulation_config)/$r/planning_description.yaml
    urdf=$(rospack find cob_hardware_config)/$r/urdf/$r.urdf.xacro
    output=$(rospack find cob_manipulation_config)/$r/default_collision_operations.yaml
    nice rosrun cob_arm_navigation planning_description_generator --urdf "$urdf" --safety "$safety" --config "$config" --output "$output" --shrink&
done
wait

    
