# glocomp_b2_ros2
All the custom code written for the unitree b2

This package depends on the unitree_ros2 package to interface with their hardware, so remember to install and source their repo
Here is the glocomp fork of the repo:

https://github.com/Glocomp-Robotics/unitree_ros2

### Setup
Add these lines into the .bashrc file of your robot/ros computer

```
source /opt/ros/humble/setup.bash
source <INSTALLATION PATH>/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

This allows your ros environment to read and compile things that uses unitree's sdk

Also a specific gotcha to keep in mind is that the /bmsstate message is of type /unitree_interfaces/msg/BmsState, but unitree_ros2 has no such msg, so please use and source the following ros2 package in your environment.(More unitree b2 specific odities still to be found)

https://github.com/Glocomp-Robotics/unitree_interfaces

### ROS Dependencies
Now that you have done the initial setup, its time to install some of the ros dependencies needed for this ros package to run

```
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-teleop-twist* ros-humble-gazebo* ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization ros-humble-image-transport-plugins
```

## How to run the robot in real world:
NEVER EVER CHANGE USE_SIM_TIME TO FALSE
<details>
    <summary>Spoiler warning</summary>
> Explanation only given if salary increment received
</details>

This starts ups the TFs, odom, clock, laserscan and cmd_vel subscriber for the robot:
```
ros2 launch glocomp_b2_ros2 robot_bringup_launch.py
```

This starts the camera republishers(Gave up trying to read the unitree videostream)
```
ros2 launch glocomp_b2_ros2 cam_launch.py
```

## How to bring up slam for the robot:
This here brings up the slammer for the robot
```
ros2 launch glocomp_b2_ros2 slam_launch.py
```

Save the map using
```
ros2 run nav2_map_server map_saver_cli -f "MAP NAME"
```

Move the robot using:
```
ros2 run teleop_twist_joy teleop_twist_joy
```
Or whatever you feel like

## Localization
```
ros2 launch glocomp_b2_ros2 localization_launch.py map:="PATH TO YOUR MAP.yaml"
```

Open another terminal
Navigation
```
ros2 launch glocomp_b2_ros2 navigation_launch.py
```


