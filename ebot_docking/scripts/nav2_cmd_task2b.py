#!/usr/bin/env python3

from typing import List
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.context import Context
from rclpy.node import Node
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from tf_transformations import quaternion_from_euler
from linkattacher_msgs.srv import AttachLink, DetachLink
from ebot_docking.srv import DockSw
from math import pi   


class Docker(Node):
    def __init__(self):
        super().__init__('Docker')
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

    def send_attach_request(self):
        self.attacher = self.create_client(AttachLink,"/ATTACH_LINK")
        while not self.attacher.wait_for_service(timeout_sec=1.0):
             self.get_logger().info('Link attacher service not available, waiting again...')
        self.get_logger().info('Connected to Attach Server')

        req = AttachLink.Request()
        req.model1_name =  'ebot'     
        req.link1_name  = 'ebot_base_link'       
        req.model2_name =  'rack1'      
        req.link2_name  = 'link'  
        self.future1 = self.attacher.call_async(req)
        rclpy.spin_until_future_complete(self,self.future1)

    def send_detach_request(self):
        self.detacher = self.create_client(DetachLink,"/DETACH_LINK")
        while not self.detacher.wait_for_service(timeout_sec=1.0):
             self.get_logger().info('Link attacher service not available, waiting again...')
        self.get_logger().info('Connected to Detach Server')
        req = DetachLink.Request()
        req.model1_name =  'ebot'     
        req.link1_name  = 'ebot_base_link'       
        req.model2_name =  'rack1'      
        req.link2_name  = 'link'  
        self.future2 = self.detacher.call_async(req)
        rclpy.spin_until_future_complete(self,self.future2)
    
    def dock_request(self,linear_dock:bool,orientation_dock:bool,distance: float,orientation: float):
        self.docker = self.create_client(DockSw,'/dock_control')
        while not self.docker.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for docker service')
        self.get_logger().info('Connected to Docker Server')
        req = DockSw.Request()
        req.linear_dock = linear_dock
        req.orientation_dock = orientation_dock
        req.distance = distance
        req.orientation = orientation
        self.future3 = self.docker.call_async(req)
        rclpy.spin_until_future_complete(self,self.future3)

    def navigate(self,goalPose:PoseStamped):
        self.navigator.goToPose(goalPose)
        i = 0
        while not self.navigator.isTaskComplete():

            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().xy_goal_toleranceinfo('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            # navigator.lifecycleShutdown()
        else:
            self.get_logger().info('Goal has an invalid return status!')


def main():
    rclpy.init()
    docker = Docker()
    

    goalPose = PoseStamped()    
    goalPose.header.frame_id = 'map'
    goalPose.header.stamp = docker.navigator.get_clock().now().to_msg()
    goalPose.pose.position.x = 0.500000
    goalPose.pose.position.y = -2.25000
    q_1 = quaternion_from_euler(0,0,3.14)
    goalPose.pose.orientation.x = q_1[0]
    goalPose.pose.orientation.y = q_1[1]
    goalPose.pose.orientation.z = q_1[2]
    goalPose.pose.orientation.w = q_1[3]

    goalPose1 = PoseStamped()
    goalPose1.header.frame_id = 'map'
    goalPose.header.stamp = docker.navigator.get_clock().now().to_msg()
    goalPose1.pose.position.x = 1.260008 - 1
    goalPose1.pose.position.y = 4.350000 + 0.25
    
    docker.navigate(goalPose1)

    docker.dock_request(True,True,0.1,3.14)

    docker.send_attach_request()

    docker.navigate(goalPose)

    docker.dock_request(False,True,0.1,3.14)
    goalPose2 = PoseStamped()    
    goalPose2.header.frame_id = 'map'
    goalPose2.header.stamp = docker.navigator.get_clock().now().to_msg()
    q_1 = quaternion_from_euler(0,0,3.14)
    docker.send_detach_request()

    goalPose2.pose.orientation.x = q_1[0]
    goalPose2.pose.orientation.y = q_1[1]
    goalPose2.pose.orientation.z = q_1[2]
    goalPose2.pose.orientation.w = q_1[3]
    docker.navigate(goalPose2)
    docker.navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()