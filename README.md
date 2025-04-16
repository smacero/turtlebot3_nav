# EECE 5554 Final Project
> This is still a work in progress!


### Cloning
git clone -b savannah https://github.com/smacero/turtlebot3_nav.git .

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### Environment Setup
In every new terminal, run:
```bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

To use `square` to drive the TB3 in TB world while running SLAM: 

```bash
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Launch Gazebo simulation:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In a new terminal, launch SLAM:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

In another terminal, run the square trajectory:
```bash
ros2 run turtlebot3_control square
```

once "Square path completed!" is printed you can save the map with:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/path/to/<mapname>
```

## Minimal Build & Dependencies
For testing trajectories, install these packages:
```bash
sudo apt install \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description
```

Then build only the necessary packages:
```bash
colcon build --packages-select \
    turtlebot3_msgs \
    turtlebot3_control \
    turtlebot3_simulations \
    turtlebot3_gazebo \
    turtlebot3_description \
    --symlink-install
```
