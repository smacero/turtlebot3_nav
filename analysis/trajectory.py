'''robot spawns in at x = -1.999464 y = -0.500002 z = 0.008510 roll = 0.000105 pitch = 0.006040

After running square.py with initial conditions of:
    standard default env configuration from launching turtlebot3_gazebo.launch.py
    and the control policy:

    if distance to goal greater than threshold: use distance control mode
        command linear and angular velo with v = kx
    else: the robot is at the goal waypoint, use heading control mode
        if heading error greater than thresh:
            cmd angular velo
        else:
            update goal to be next waypoint in the list
            if at last waypoint:
            # idea: add time delay
            send zero commands to x and y 
final pose:
x = -1.985767
y = -0.448033
z = 0.008526
roll = 0.000062
pitch = 0.005737
yaw = -0.092938

'''
