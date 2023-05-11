# Useful commands

### Create new package

```ros2 pkg create --build-type ament_python [package_name]```

### Build packages

```colcon build --packages-select [package_name]```
```source install/setup.bash```

### Launch world

```ros2 launch warehouse_robot_spawner_pkg gazebo_world.launch.py```

### Run package

```ros2 run [package_name] [entry_point]```
