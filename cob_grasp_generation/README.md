### MANUAL GENERATION OF GRASP TABLE

#### 1. Provide a collision mesh of the target object (as `*.stl`)
Most likely there already is a collada mesh (`*.dae`) available in `cob_gazebo_objects`. Thus you simply have to convert the mesh from `*.dae` to `*.stl` using MeshLab for example.
(1) Open MeshLab -> File -> Import Mesh ... -> select respective mesh `[object_name].dae` from `cob_simulation/cob_gazebo_objects/media/models`
(2) File -> Export Mesh As ... -> Save it as `[object_name].stl` within `cob_manipulatiuon/cob_grasp_generation/files/meshes`

#### 2. Prepare a new grasp table
You need to provide a `*.csv` file of the name `[gripper_type]_[object_name].csv` at `cob_manipulation/cob_grasp_generation/files/database/[object_name]`, where `[object_name]` is the same as for the collision mesh and `[gripper_type]` could e.g. be either `sdh` or `sdhx` (cob4-gripper).
The `*.csv` file has the following layout (columns):

| id | object | *[gripper_joint_0]*  | *[gripper_joint_n]* | direction | qw | qx | qy | qz | pos-x | pos-y | pos-z | eps_l1 | vol_l1 |   |
|:--:|:------:|:------------------:|:-----------------:|:---------:|:--:|:--:|:--:|:--:|:-----:|:-------:|:-----:|:------:|:------:|:-:|
|0|*[object_name]*|*[joint_pos_0]*|*[joint_pos_n]*|*[DIRECTION]*|*[qw]*|*[qx]*|*[qy]*|*[qz]*|*[pos-x]*|*[pos-y]*|*[pos-z]*|*[eps_l1]*|*[vol_l1]*|0|

where `id` is the grasp_id, `object` is `[object_name]`, followed by one column for each joint of the gripper specifying the joint configuration of the gripper when grasping the object (with `gripper_joint_0` to `gripper_joint_n` being the gripper's joint_names), `direction` can be used to specify e.g. as [TOP|SIDE|BOTTOM|...] grasp, the grasp pose of the gripper reference coordinate system with respect to the object reference coordinate system (with [qw, qx, qy, qz] specifying the orientation as a quaternion and [pos-x, pos-y, pos-z] specifying the position in [mm]), `eps_l1` and `vol_l1` being grasp metrics e.g. force closure that can be used as grasp quality metric for sorting the grasps.

Notes:
 - each ros specifies another grasps (make sure to increase `id` accordingly)
 - the quaternion needs to be normalized, i.e. sqrt(qw² + qx² + qy² + qz²)!=1.0 (you might need to increase floating point precision to about 5 decimal digits. e.g. (0.0, 0.707, 0.0, 0.707) will throw an error; use (0.0, 0.707107, 0.0, 0.707107) instead!
 - you can initialize a grasptable by copying from an existing database entry
 
#### 3. Use blender for generating initial grasp poses easily
 (1) Open the Blender
 (2) Clear the scene by removing all objects: press `a` (select all the objects), `Entf` (delete), `Enter` (confirm)
 (3) Load the object: File -> Import -> STL (.stl) -> select `[object_name].stl` from `cob_manipulatiuon/cob_grasp_generation/files/meshes` (see step (1))
 (4) Load the gripper: File -> Import -> STL (.stl) -> select `[gripper].stl`, e.g. `palm.stl` from `cob_common/cob_descriptions/meshes/cob4_gripper` for sdhx (cob4-gripper)
 (5) Move the gripper to various grasp poses (**do not move the object**) by either using
     - ```g+x,g+y,g+z: for translation along x,y,z```
     - ```r+x,r+y,r+z: for rotation around x,y,z```
     - ```enter values directly```
 (6) Transfer the values from the Blender (position in [m], use the Quaternion-Display!) to the grasp table (pos in [mm]

#### 4. Visualization of the grasps  in rviz
 (1) `roslaunch cob_grasp_generation show_grasp_rviz.launch gripper:=[gripper_type]`
 (2) `rosrun cob_grasp_generation show_grasps_rviz_client.py` (then enter [object_name] and [gripper_type] when prompted)

### 5. Tune grasps 
Optimize the grasp entries in the table by modifying the grasp pose and/or grasp joint config based on the rviz visualization. Add more grasps
