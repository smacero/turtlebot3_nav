# EECE 5554 Final Project: High Level Turtlebot3 Controller
> This is still a work in progress! The `fast_hash` node is being repurposed in the future for another controller, but for now `square` is up and running. Please email me at savannahmacero@gmail.com if you give it a shot and find any issues!


### About

The goal of this project was to create an alternative package to `turtlebot3_teleop` that provides the user with an automated control method. Using the TB World environment, the user can generate a map with `turtlebot3_cartographer` while our package, `turtlebot3_control`, generates and follows a closed loop trajectory using simple waypoint following.

This repository includes `turtlebot3_control`'s dependencies `turtlebot3`, `turtlebot3_msgs`, and `turtlebot3_simulations`. Before cloning this repository, you still need to install the system requirements for running ROS2 and Gazebo. 

Alternatively, if you have a pre-existing workspace you'd like to use you can simply download `turtlebot3_control` located [here](src/) and build in your workspace. 

This setup process is adapted from the standard TB3 tutorial found at https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup.

## Install dependencies

Install Gazebo, Cartographer, and Nav2

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

Clone this repository in your root directory and set up ROS environment

```bash
source /opt/ros/humble/setup.bash
git clone -b savannah https://github.com/smacero/turtlebot3_nav.git
sudo apt install python3-colcon-common-extensions
cd ~/turtlebot3_nav
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Tips
In every new terminal, run:
```bash
source ~/.bashrc # or `source install/setup.bash` depending on your use 
export TURTLEBOT3_MODEL=burger
```

## Tutorial 
To use `square` to drive the TB3 in TB world while running SLAM: 

Launch Gazebo (don't forget to source and export):

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Prepare two terminals and run the following commands in this order:

1. Launch cartographer to initialize the SLAM session:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
2. Run the controller to drive TB3 around:

```bash
ros2 run turtlebot3_control square
```

Once "Square path completed!" is printed you can save the map with:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/path/to/<mapname>
```


