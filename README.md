# RoboStud
For use in robotics studio 1

## How to link package to ros2_ws/src
```bash
ln -s /path/to/your/package .
```

## How to compile
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

## Launch teleop
```bash
export TURTLEBOT3_MODEL=waffle_pi 
ros2 run turtlebot3_teleop teleop_keyboard
```



