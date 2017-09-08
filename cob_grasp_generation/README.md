### MANUAL GRASP GENERATION

#### 1. export mesh: .dae->.stl using MeshLab
(1) open MeshLab -> Import Mesh -> the location of the collada mesh (cob_simulation/cob_gazebo_objects/media/models/[object_name].dae)
(2) Export  Files as.. -> cob_manipulatiuon/cob_grasp_generation/files/meshes/[object_name].stl (Form: .stl)
 
#### 2. prepare grasp table: sdh_[object_name].csv/sdhx_[object_name].csv
 (1) cob_manipulation/cob_grasp_generation/files/database: 
 (2) copy one ordner, paste, change the name ([object_name])
 (3) change the directory to the ordner ([object_name]),change the name (sdh_[object_name].csv/sdhx_[object_name].csv)
 (4) open the dokument (sdh_[object_name].csv/sdhx_[object_name].csv), delete the 3,4,... lines
 
#### 3. use blender: the position in the gripper
 (1) open the Blender
 (2) a (select all the objects) -> Entf (delete)
 (3) load the object: file -> import -> .stl -> the location of the .stl document (cob_manipulatiuon/cob_grasp_generation/files/meshes/[object_name].stl)
 (4) load the gripper: file -> import -> .stl -> the location of the .stl document (cob_common/cob_descriptions/meshes/cob4_gripper/palm.stl)
 ```g+x,g+y,g+z: movement along x,y,z```
 ```r+x,r+y,r+z: rotation around x,y,z```
 ```change the values from Transform directly```
 (5) move the gripper to the right position (do not move the object)
 (6) transfer the values(meter) in Tranform to [object_name].csv(milimeter)
 ```take care of the normalization```

#### 4. visualize in rviz: check collisions
 (1) roslaunch cob_grasp_generation show_grasp_rviz.launch gripper:=sdh (or sdhx)
 (2) rosrun cob_grasp_generation show_grasps_rviz_client.py (then enter object_name and gripper_type)
 (3) check the collisions, change the parameter in .csv (including the gripper parameter)
 (4) generate more points
