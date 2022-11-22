# create3_controller

## configuration 
config ``config/params.yaml`` file before execution to avoid runtime errors.
Two file paths need to be changed based on your absoulate folder paths. 

## necessary commands

#### compile node
```bash 
colcon build --packages-select create3_controller
```

#### execute node 
```bash
ros2 launch create3_controller create3_controller_launch.yaml 
```

#### gazebo simulation 
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py x:=-0.5 y:=1.76
```

##### disable safety limits on gazebo 
```bash
ros2 param set /motion_control safety_override full
```



## create ros package 

```bash 
ros2 pkg create --build-type ament_cmake --node-name create3_path_follower create3_path_follower \
    --dependencies rclcpp tf visualization_msgs nav_msgs geometry_msgs irobot_create_msgs
```