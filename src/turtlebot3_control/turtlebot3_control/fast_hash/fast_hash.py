'''brainstorm:
inuitively, given the geometry of turtlebot world, you might guess that following one continous 
hash trajectory would generate a complete map, but thats super long and would take a long time. 
but what if we went fast? I dont think the turtle would tip over with reasonably fast speeds. 
'''

import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FastHash(Node):
    def __init__(self):
        super().__init__('fast_hash') # shoudl match .py file name
        self.get_logger().info('Starting fast hash path...')
            
        # Define square path waypoints
        self.waypoints = [
            (-2.0, -0.5),    # Starting/spawn position
            (-1.5, -1.5),    # First corner
            (1.5, -1.5),     # Second corner
            (1.5, -0.5) ,    # line up at column 1
            (-2.0, -0.5),    # end of column 1
            (-2.0, 0.5),     # line up at column 2
            (1.5, 0.5),       # end of column 2
            (1.5, 1.5),      # line up at top left edge of env
            (0.5, 1.5),      # line up at row 1
            (0.5, -1.5),       # end of row 1 
            (-0.5, -1.5),    # line up at row 2
            (-0.5, 1.5),      # end of row 2
            (-1.5, 1.5),     # line up at bottom left edge of env
            (-2.0, -0.5),    # return to start
        ]
        self.current_waypoint = 0

        # Robot state
        self.position = Point(x=0.0, y=0.0, z=0.0) # args are this bc Point needs floats 
        self.heading = 0.0
        self.goal_position = Point(x=0.0, y=0.0, z=0.0) 
        self.goal_heading = 0.0

        # Control parameters
        self.linear_speed = 2
        self.angular_speed = 0.5
        self.distance_threshold = 0.1
        self.heading_threshold = 0.1

        # ROS setup
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Start moving
        self.timer = self.create_timer(0.1, self.control_loop)
        self.set_next_goal()

        # Collision detection parameters
        self.collision_threshold = 0.3
        self.has_collided = False
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.start_position_checked = False
        self.expected_start = (-2.0, -0.5)  # Starting/spawn position
        self.start_threshold = 0.3 

    # Make sure twist vals are floats for Point just in case
    def create_twist_msg(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        return cmd
    
    def scan_callback(self,msg):
        # check for closest object in scan
        min_distance = min(msg.ranges)
        if min_distance < self.collision_threshold and not self.has_collided:  # Only trigger once
            self.has_collided = True
            self.get_logger().info('Collision detected! Stopping TB3..')
            self.e_stop()
            self.get_logger().info('Shutting down node...')
            self.destroy_node()  # Stop the node
            rclpy.shutdown()     # Exit ROS2
            return


    def e_stop(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = float(0.0)
        stop_cmd.angular.z = float(0.0)
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('E-stop activated...')

    def set_next_goal(self):
        next_point = self.waypoints[self.current_waypoint]
        self.goal_position.x = float(next_point[0])
        self.goal_position.y = float(next_point[1])
        
        # Calculate heading to next waypoint
        next_idx = (self.current_waypoint + 1) % len(self.waypoints)
        next_next_point = self.waypoints[next_idx]
        self.goal_heading = math.atan2(
            next_next_point[1] - next_point[1],
            next_next_point[0] - next_point[0]
        )
        
    def control_loop(self):
        # Calculate distance to goal
        if self.has_collided:   # check each time step for collision status
            return
        dx = float(self.goal_position.x - self.position.x)
        dy = float(self.goal_position.y - self.position.y)
        distance = math.sqrt(dx*dx + dy*dy)
        
        cmd = Twist()
        
        if distance > self.distance_threshold:
            # Move to point
            angle_to_goal = math.atan2(dy, dx)
            angle_error = angle_to_goal - self.heading
            
            # Normalize angle so robot turns shortest way
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi
            
            cmd.angular.z = float(self.angular_speed * 5.0 * angle_error) # Slow down as goal ange is approached: maybe try changing this 
            cmd.linear.x = float(min(self.linear_speed * distance, 2.0)) # trying thresholding at 2 m/s to see if itll go faster
            
        else: # Once at waypoint, face the next one
            # At waypoint - rotate to final heading
            heading_error = self.goal_heading - self.heading
            while heading_error > math.pi: heading_error -= 2*math.pi
            while heading_error < -math.pi: heading_error += 2*math.pi
            
            if abs(heading_error) > self.heading_threshold: 
                cmd.angular.z = self.angular_speed * 5.0 * heading_error
                # cmd.angular.z = self.angular_speed
            else:
                # Move to next waypoint
                self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints) # update waypoint index
                if self.current_waypoint == 0:
                    self.get_logger().info('Hash path completed!')
                    self.destroy_node()
                    return
                self.set_next_goal()
        if not self.has_collided:
            self.cmd_vel_pub.publish(cmd)
    
    def euler_from_quaternion(self, q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def odom_callback(self, msg):
        # Explicitly set Point as floats bc of type issue
        self.position.x = float(msg.pose.pose.position.x)
        self.position.y = float(msg.pose.pose.position.y)
        self.position.z = float(msg.pose.pose.position.z)
        _, _, self.heading = self.euler_from_quaternion(msg.pose.pose.orientation)

        if not self.start_position_checked:
            dx = float(self.position.x - self.expected_start[0])
            dy = float(self.position.y - self.expected_start[1])
            distance_from_start = math.sqrt(dx*dx + dy*dy)
            
            if distance_from_start > self.start_threshold:
                self.get_logger().error(
                    f'\nTB3 is too far from start'
                    f'\nExpected: {self.expected_start}'
                    f'\nActual: ({self.position.x:.2f}, {self.position.y:.2f})'
                    f'\n1. Click "Edit" in Gazebo window'
                    f'\n2. Select "Reset Model Poses"'
                    f'\n3. Run this node again'
                )
                self.destroy_node()
                rclpy.shutdown()
                return
            self.start_position_checked = True 



def main(args=None):
    rclpy.init(args=args)
    node = FastHash()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get.logger().info('Keyboard interrupt')
    except Exception as e:
        import traceback
        tb = traceback.extract_tb(e.__traceback__)
        filename, line, func, text = tb[-1]  # Get the last frame
        node.get_logger().error(
            f'\nError Type: {e.__class__.__name__}'
            f'\nLocation: {filename}:{line} in {func}()'
            f'\nCode: {text}'
            f'\nError: {str(e)}'
        )
    finally:
        node.e_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()