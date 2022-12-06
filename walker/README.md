## TurtleBot Walker

A ROS 2 package developed for turtlebot to depict walker behavior. The robot will explore it's enviroment and avoid any obstacle. In case the robot at in a distance to an obstacle which is less than a threshold, it will rotate until path ahead of it is clear.


## Dependencies
<ul>
<li> Ubuntu 20.04 </li>
<li> ROS 2 Foxy </li>
<li> Turtlebot3 package for ROS 2 </li>
<li> Gazebo </li>
</ul>

## Build instructions
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## Cloning the repository
```
git clone https://github.com/sj0897/Walker.git
```

## Build the workspace
```
cd ~/ros2_ws
colcon build --packages-select walker
```

## Set turtlebot3 variable for model
```
echo  "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

## Run Instructions

## Using ROS Bag record
```
. install/setup.bash
ros2 launch walker walker.launch.py record:=True
  ```

## Inspect the ROS bag with following command
```
ros2 bag info <path_to_rosbag_record_file>
```

## To play with ROS bag, close gazebo and exit the launch file. 
Open a new terminal
```
cd ~/ros2_ws
. install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Open another new terminal
```
cd ~/ros2_ws
. install/setup.bash
ros2 bag play <path_to_rosbag_record_file>
``