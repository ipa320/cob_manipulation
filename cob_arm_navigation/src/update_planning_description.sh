#!/bin/bash
if [ $# -ne 1 ]
then
    echo "Usage: update_planning_description ROBOT"
    echo "It may take hours to finish!"
    exit 0
elif [ $1 -eq "ALL" ]
then
    robots=""
    for d in `rospack find cob_manipulation_config`/*/planning_description.yaml; do robots="$robots $(basename $(dirname $d))"; done    
else
    robots=$1
fi
trap "kill 0" SIGINT SIGTERM EXIT
roscore&
for r in $robots
do
    echo $r
    rosrun cob_arm_navigation planning_description_generator --input "$(rospack find cob_hardware_config)/$r/urdf/$r.urdf.xacro" --type VerySafe --output "$(rospack find cob_manipulation_config)/$r/planning_description.yaml"
done


    