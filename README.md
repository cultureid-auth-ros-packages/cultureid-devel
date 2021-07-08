A shell package to execute robot bringup, SLAM, localisation, and similar functionalities needed in project cultureid. This is the centralised locus of robot-centric packages and launchers.

# Packages needed for the operation of ROBOTNIK RB1:

```
git clone git@github.com:robotics-4-all/cultureid-rb1-packages.git
```
# Packages needed for the operation of turtlebot:

```
git clone git@github.com:robotics-4-all/cultureid-turtlebot-packages.git
```

# Packages needed by both robots:

```
git clone git@github.com:robotics-4-all/cultureid-devel.git
git clone git@github.com:robotics-4-all/relief-amcl-mod.git
git clone git@github.com:robotics-4-all/cultureid-rfid-detection.git
git clone git@github.com:robotics-4-all/cultureid-support-files.git
git clone git@github.com:robotics-4-all/cultureid-rfid-antennas-poses-logger.git
git clone git@github.com:robotics-4-all/cultureid-rfid-visualisation.git
git clone git@github.com:KumarRobotics/waypoint_navigation_plugin.git
```
---

# Directory `launch`
Houses launchers for robot bringup, simulation environment, mapping, localisation, and navigation. The entry point for launching the total operation of the above is prefixed by `avanti_` and differentiated by the nature of the environment that a robot is placed in: `avanti_simulation.launch` launches the `gazebo` simulator and poses the robot at `(real_initial_pose_x, real_initial_pose_y, real_initial_pose_yaw)` in it, whereas `avanti_live.launch` launches everything else except the surrounding reality.

## Launching SLAM
```
roslaunch cultureid_devel avanti_simulation.launch task:=mapping slam_alg:=SA slam_resolution:=SR
```
where
- `SA = {karto|gmapping|rtabmap}`
- `SR` is the resolution of the 2D grid in meters per cell, and usually set from 0.01 to 0.1 m/cell


## Launching `amcl` localisation
```
roslaunch cultureid_devel avanti_simulation.launch task:=localisation
```

### Additional options
```
roslaunch cultureid_devel avanti_simulation.launch task:={localisation|mapping} gazebo_world:=GW robot_type:=RT gazebo_gui:={true|false} num_cameras:=NC gp:=GP lp:=LP use_explorer:={true|false} do_viz:={true|false}
```
- `GW` is the name of the gazebo_world in simulation. The world of the same name should already be in directory `worlds`
- `RT={turtlebot|rb1}`
- `NC={1|2|3}` for `robot_type = rb1` and `NC = 1` for `robot_type = turtlebot`
- `GP={navfn|globalplanner|sbpl}`; defaults to `globalplanner`
- `LP={dwa|eband|teb}`; defaults to `teb`

### Additional instructions concerning mapping
- Upon completion of mapping, issue `$ rosrun map_server map_saver -f name_of_map`. The map's resolution (and corresponding world in the case of simulation) should be recorded in the `avanti_` launchers. After saving the map, its origin should be commented-out and replaced by `[0,0,0]` in its corresponding `.yaml` file; the original origin could be used as the robot's starting position in simulation by providing amcl with the same origin but with inverted signs.
- The name of the map should reflect the map's resolution, e.g. the file `map_csal.pgm` with resolution 0.05 m/cell should be renamed to `map_csal_0.05.pgm`. This is done for automatic detection lower down in the hierarchy of launchers and reusability purposes.
- For constructing a 3D map with `rtabmap` the process is the following:

    1. `rtabmap ~/.ros/xxx.db` --> export to xxx.ply using default settings. remember to check `meshing`; click `regenerate clouds` and set the maximum depth (max ~4.0)
    2. `./binvox -e xxx.ply` --> exports a xxx.binvox (best grab binvox from [here](https://www.patrickmin.com/binvox/); it's the latest version (10 May 2019).)
    3. `./binvox2bt xxx.binvox` --> exports a xxx.binvox.bt (Compile [binvox2bt.cpp](https://github.com/OctoMap/octomap/blob/devel/octomap/src/binvox2bt.cpp) with `g++ binvox2bt.cpp -loctomap -loctomath -o binvox2bt`; needs `liboctomap-dev`)
    4. `octovis xxx.binvox.bt` (sudo apt install ros-kinetic-octovis)

    References: [here](https://answers.ros.org/question/321296/it-is-possible-to-create-a-map-with-rtabmap-using-the-laser-and-obtaining-3d-point-cloud-at-the-same-time/) and [here](http://ros-developer.com/2017/11/29/create-octomap-tree-mesh/)


---

### Directory `launchers/exploration`
Provides launchers for frontier exploration. The functionality is unfinished and untested.

### Directory `launchers/localisation`
Provides wrapping launchers for ROS localisation packages `amcl` and `relief_amcl_mod`.

### Directory `launchers/mapping`
Provides wrapping launchers for loading maps and launching ROS SLAM packages `gmapping`, `karto`, and `rtabmap`. The latter may run in a configuration of one, two, or three cameras.

### Directory `launchers/navigation`
Provides wrapping launchers for launching `move_base` and teleoperation in a per-robot basis. Additionally, the `follow_waypoints.launch` launcher launches the ROS node that subscribes to incoming lists of targets for navigation and publishes them to `move_base`. Very handy for defining intermediate targets to a more distant target, or executing the same path over and over (can be set via a `.csv` file loaded at runtime).

### Directory `launchers/robot`
Provides wrapping launchers for launching `gazebo`, `rb1` in simulation and in reality, and `turtlebot` in simulation and reality. Usually one would execute
```
$ roslaunch cultureid_devel {rb1|turtlebot}_bringup_{live|simulation}.launch
```
before launching `avanti_{live|simulation}.launch` in localisation or mapping mode.

---

# Directory `configuration_files`
Provides configuration files for the necessary `{common, local, global} costmap`, `{local,global} planners`, `move_base`, `localisation`, `mapping`, and `rviz` components. These params are loaded through launchers. Their values are modified therein in a per-robot-type basis.

---

# Directories `src` and `include`

Some helper code, e.g. to test the accuracy of the odometry, provide a quick quaternion-to-rpy transformation, and most notably, the code for `follow_waypoints`, which, provided the `waypoint_navigation_plugin`, helps assigning consecutive targets to be reached by `move_base`

---

# Directory `maps`
Houses all OGM maps of all environments, either simulated or real

---

# Directory `worlds`
Houses `gazebo` worlds

---

# Directory `msg`
Houses a custom message for quick inspection of orientation (quaternion-to-rpy)

---
# Live the full cultureid experience

## In robot `rb1`

Using SSH, in separate terminals execute
```
$ roslaunch cultureid_devel rb1_bringup_live.launch
$ roslaunch cultureid_devel avanti_live.launch task:={mapping|localisation} [slam_resolution:=0.XX]
$ roslaunch cultureid_devel rb1_teleop.launch
$ roslaunch cultureid_rfid_antennas_poses_logger avanti_log.launch
$ roscd cultureid_rfid_detection/application; make clean && make x86 && bin/speedwayr_x86 192.168.0.190 001625127C5D
$ roscd cultureid_rfid_detection/application; make clean && make x86 && bin/speedwayr_x86 192.168.0.192 001625127C5F
$ roslaunch cultureid_rfid_detection localise_rfid_tags.launch

```
locally run `export ROS_MASTER_URI=http://192.168.0.106:11311` to bringup rVIZ to your own computer

## In robot `turtlebot`

Using SSH, in separate terminals execute
```
$ roslaunch cultureid_devel turtlebot_bringup_live.launch
$ roslaunch cultureid_devel avanti_live.launch task:={mapping|localisation} [slam_resolution:=0.XX]
$ roslaunch cultureid_devel turtlebot_teleop.launch
$ roslaunch cultureid_rfid_antennas_poses_logger avanti_log.launch
$ roscd cultureid_rfid_detection/application; make clean && make x86 && bin/speedwayr_x86 192.168.2.10 001625127C5A
$ roslaunch cultureid_rfid_detection localise_rfid_tags.launch

```

locally run `export ROS_MASTER_URI=http://192.168.16.196:11311` to bringup rVIZ to your own computer
