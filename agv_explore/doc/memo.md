
## Create Map Point Cloud

### @ myagv
$ roslaunch agv_explore myagv_livox_hw.launch
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

### @ livox vim4

$ roslaunch fast_lio mapping_mid360.launch rviz:=false livox_driver:=true

** rename the .pcd file under fast_lio/PCD/


## Convert to 2D Map

### @ livox vim4

$ roslaunch fast_lio pcd2map.launch file_name:=`rospack find fast_lio`/PCD/8-326_scans.pcd min_height:=-0.1 map_name:=map-name  # 8-326

### @ myagv

$ sftp leus@1921.68.1.108 # get map

** copt map files under gas_ditribution/maps


## Localization & Navigation

### @ myagv
$ roslaunch agv_explore aibot_bringup.launch livox_hw:=true map_file_shorthand:=livox-8-326.yaml
# with exising path map: 


### @ livox vim4
$ roslaunch fast_lio_localization localization_mid360.launch map:=`rospack find fast_lio`/PCD/8-326_scans.pcd livox_driver:=true headless:=true

### @ local
$ roslaunch agv_explore rviz_display.launch


## [Additional] Generate Existing Path Mapg
### @ myagv
$ rosrun agv_explore existing_path_generator

### @ myagv
$ rosrun agv_explore existing_path_generator
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py # keyboard
$ rosrun map_server map_saver -f xxx-existing-path-map map:=/path_map

## Patrol Demo

$ roslaunch agv_explore gas_patrol.launch plan_file_short_hand:=::.yaml # auto waypoint mission
