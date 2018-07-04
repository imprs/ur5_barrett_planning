# ur5_barrett

There are 3 packages in the group.

  - ur5_barrett_description -> urdf/xacro and mesh files
  - ur5_barrett_moveit_config -> moveit config files
  - ur5_barrett_planning -> planning interface in C++

### Pre-requisites

  - ROS (recommended: indigo/kinetic) http://wiki.ros.org/ROS/Installation
  - moveit! http://moveit.ros.org/install/

### Make
  - Create a workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace
  - Unzip src.zip into the workspace
  - Build the workspace with command: 
    ```
    $ catkin_make
    ```
  - Source your new setup.*sh file: 
    ```
    $ source devel/setup.bash
    ```

### Usage/Launch
```
    $ roslaunch ur5_barrett_planning ur5_barret_bringup.launch
```
  - The above command will launch all required nodes and open rviz for the visulization.
  - Once all processes have been started, it will ask to input no_of_poses into the terminal.
  - At the end, poseInfo.csv file will be generated in the ur5_barrett_planning package.
