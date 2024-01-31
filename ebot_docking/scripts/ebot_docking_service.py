#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback,10,callback_group=self.callback_group)
        self.odom_sub = self.create_subscription(Imu, '/imu', self.imu_callback,10,callback_group=self.callback_group)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback,10,callback_group=self.callback_group)
        self.ultrasonic_rr_sub = self.create_subscription(Range,'/ultrasonic_rr/scan',self.ultrasonic_rr_callback,10,callback_group=self.callback_group)


        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        #
        self.cmd = self.create_publisher(Twist,'/cmd_vel',10)

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.angle_to_rotate = 0
        # self.robot_pose = [0,0,0]
        self.yaw = 0
        self.dock_aligned = False
        self.is_linear_dock = False
        self.kp = 2
        self.kp2 = 1.5
        self.msg = Twist()
        #timer object

        self.controller_timer = self.create_timer(0.1, self.controller_loop,callback_group=self.callback_group)
        

    # Callback function for odometry data
    def imu_callback(self, msg:Imu):
        # # Extract and update robot pose information from odometry message
        # self.robot_pose[0] = msg.
        # self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        self.yaw = euler_from_quaternion(orientation_list)[2]
       
        # self.get_logger().info('odom'+str(yaw))
    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range
        # self.get_logger().info('rl: '+str(self.usrleft_value)        
            # Callback function for the right ultrasonic sensor
    #commands
    #
    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range
        # self.get_logger().info('rr:'+str(self.usrright_value))

    # Utility function to normalize angles within the range of 0 to 2π (OPTIONAL)
    def normalize_angle(self,angle):
        if(angle>=0):
            return angle
        else:
            return 6.28 - abs(angle)
        pass

    # Main control loop for managing docking behavior

    def controller_loop(self):


        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        
        if self.is_docking:
            # if self.robot_pose[2]>=0:
            #     angle_rotated = abs(self.robot_pose[2]-self.init_yaw)
            # elif self.robot_pose[2]<0:
            #     angle_rotated = (6.28-abs(self.robot_pose[2])-abs(self.init_yaw))
            
            # self.get_logger().info(' angle rotated: '+str(self.robot_pose[2]))
            
            error = self.target_yaw-self.normalize_angle(self.yaw)
            self.get_logger().info('error: '+str(error))
            amp_error = self.kp*error
            self.msg.angular.z = amp_error
            # self.get_logger().info(str(self.msg.angular.z))
            self.cmd.publish(self.msg)

            if abs(error)<0.005:
                self.msg.angular.z = 0.0
                self.get_logger().info('done')
                # if self.is_linear_dock == False:
                #     self.dock_aligned = True
                self.cmd.publish(self.msg)
                self.is_docking=False
                if self.is_linear_dock == False:
                    self.dock_aligned = True
                
        if (self.is_linear_dock == True and self.is_docking == False):
            curr_value = (self.usrleft_value+self.usrright_value)/2
            # self.get_logger().info(str(curr_value))
            error_2 = self.target_distance-curr_value
            self.get_logger().info(str(error_2))

            self.msg.linear.x=error_2*self.kp2
            self.cmd.publish(self.msg)
            if round(abs(error_2),3)==0.001:
                self.msg.linear.x = 0.0
                self.cmd.publish(self.msg)
                self.is_linear_dock = False
                self.dock_aligned = True

    # Callback function for the DockControl service
    def dock_control_callback(self, request:DockSw, response:DockSw):
        # Extract desired docking parameters from the service request
        #
        #

        # Reset flags and start the docking process
        #
        #

        # Log a message indicating that docking has started
        self.target_yaw = self.normalize_angle(request.orientation)
        self.dock_aligned = False
        self.is_linear_dock = request.linear_dock
        self.is_docking = request.orientation_dock
        self.target_distance = request.distance

        self.get_logger().info(str(self.target_yaw))                                    
        self.get_logger().info("Docking started!")


        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())
        
        # self.theta = abs(self.usrleft_value-self.usrright_value)/0.09
        # self.angle_to_rotate = math.atan(self.theta)
        # self.angle_to_rotate = 0.262
        # self.get_logger().info('angle to rotate: '+str(self.angle_to_rotate))
        # self.init_yaw = self.robot_pose[2]
        # self.get_logger().info('initial yaw'+str(self.init_yaw))
        # Wait until the robot is alTwistigned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for  alignment...")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        self.is_docking = False
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    print('start')
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
