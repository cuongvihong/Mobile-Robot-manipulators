# myrobot_cartographer_slam

ROS 2 Cartographer SLAM package for TF trees of the form:
- world -> rX_odom
- rX_odom -> rX_mobilebase_link
- rX_mobilebase_link -> rX_lidar_link

## Included configs
- `r1_2d.lua`
- `r2_2d.lua`
- `r3_2d.lua`

## Launch files
- `cartographer_one_robot.launch.py`
- `cartographer_multi_robot.launch.py`
