# Scan Matching Localiser

```bash
sudo apt install libopencv-dev
sudo apt install libyaml-cpp-dev
```
## How to Build the Package

Navigate to the root of your ROS2 workspace:
```bash
cd ~/ros2_ws
```

## Build the package:
```bash
colcon build --packages-select scan_matcher_package
```

## Run the Node
```bash
ros2 run sprint_2_localiser scan_matching_localiser ~/ros2_ws/src/sprint_2_localiser/src/small_warehouse.yaml
```

