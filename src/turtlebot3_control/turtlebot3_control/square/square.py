# TODO: store initial pose at beginning and once path is complete, 
# check if robot is at the same position as the initial pose and print error
# check orientation and adjust to be same as initial pose
'''ISSUES
lin velocity is approx zero randomly during segments
ang velocity control is still not great
'''
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
# from scipy.spatial.transform import Rotation as R

# robot spawns in at x = -1.999464 y = -0.500002 z = 0.008510 roll = 0.000105 pitch = 0.006040


class Square(Node):
    def __init__(self):
        super().__init__('square') # shoudl match .py file name
        self.get_logger().info('Starting square path...')
        
        # Define square path waypoints
        # self.waypoints = [
        #     (-2.0, -0.5),    # Starting/spawn position
        #     (-1.5, -1.5),    # First corner
        #     (1.5, -1.5),     # Second corner
        #     (1.5, 1.5),      # Third corner
        #     (-1.5, 1.5),     # Fourth corner
        #     (-2.0, -0.5)     # Return to start
        # ]

        # Use hash waypoints to get results since hash is broken right now lol
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
        self.position = Point()
        self.heading = 0.0
        self.goal_position = Point()
        self.goal_heading = 0.0

        # Control parameters
        self.linear_speed = 2
        self.angular_speed = 0.7
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
        dx = self.goal_position.x - self.position.x
        dy = self.goal_position.y - self.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        cmd = Twist()
        
        if distance > self.distance_threshold:
            # Move to point
            angle_to_goal = math.atan2(dy, dx)
            angle_error = angle_to_goal - self.heading
            
            # Normalize angle so robot turns shortest way
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi
            
            cmd.angular.z = self.angular_speed * angle_error # Slow down as goal ange is approached: maybe try changing this 
            cmd.linear.x = min(self.linear_speed * distance, 0.2) # keep this slow since we expect map qual to be lower with this trajectory
            
        else: # Once at waypoint, face the next one
            # At waypoint - rotate to final heading
            heading_error = self.goal_heading - self.heading
            while heading_error > math.pi: heading_error -= 2*math.pi
            while heading_error < -math.pi: heading_error += 2*math.pi
            
            if abs(heading_error) > self.heading_threshold: 
                cmd.angular.z = self.angular_speed * heading_error
            else:
                # Move to next waypoint
                self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints) # update waypoint index
                if self.current_waypoint == 0:
                    self.get_logger().info('Square path completed!')
                    self.destroy_node()
                    return
                self.set_next_goal()
                
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
        self.position = msg.pose.pose.position
        _, _, self.heading = self.euler_from_quaternion(msg.pose.pose.orientation)

def main(args=None):
    rclpy.init(args=args)
    node = Square()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()