# RoboStud
For use in robotics studio 1

## How to link package to ros2_ws/src
```bash
ln -s /path/to/your/package .
```

## How to compile
### Compile All
```bash
colcon build --symlink-install
```
### Compile single package
```bash
colcon build --packages-select <package_name>
```

## Launch Turtlebot3 World
```bash
export TURTLEBOT3_MODEL=waffle_pi 
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Launch Turtlebot3 House
```bash
export TURTLEBOT3_MODEL=waffle_pi 
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

## Launh Rviz
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

## Launch Cartographer
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

## Launch GMapping
```bash
ros2 launch turtlebot3_slam turtlebot3_slam.launch.py slam_methods:=gmapping use_sim_time:=True
```

## Launch teleop
```bash
export TURTLEBOT3_MODEL=waffle_pi 
ros2 run turtlebot3_teleop teleop_keyboard
```
## RQT Plot for graph
```bash
ros2 run rqt_plot rqt_plot
```
Note: Need to be specific: /odom/pose/pose/position

## When in doubt
```bash
source ~/.bashrc
```
```bash
source ~/ros2_ws/install/setup.bash
```


