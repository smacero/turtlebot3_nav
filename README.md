# EECE 5554 Final Project
Instructions: 
`colcon build --symlink-install`

`source ~/.bashrc`

`export TURTLEBOT3_MODEL=burger`

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

`ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`

`ros2 run turtlebot3_control square`

once "Square path completed!" is printed you can save the map with:

`ros2 run nav2_map_server map_saver_cli -f ~/path/to/<mapname>`