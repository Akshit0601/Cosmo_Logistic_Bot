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
# from linkattacher_msgs.srv import AttachLink, DetachLink
from ebot_docking.srv import DockSw
from math import pi,cos,sin   
import yaml
from usb_relay.srv import RelaySw
from time import sleep
from nav_msgs.msg import Odometry

def load_yaml(path):
    with open(path,"r+") as file:
        file_handle = yaml.safe_load(file)
        return file_handle


class Docker(Node):
    def __init__(self):
        super().__init__('Docker')
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback,10)
        self.odom_init = True
        

    def odometry_callback(self,msg:Odometry):
        if self.odom_init:
            self.initial_pose.pose.position = msg.pose.pose.position
            self.initial_pose.pose.orientation = msg.pose.pose.orientation
            self.navigator.setInitialPose(self.initial_pose)
            self.navigator.waitUntilNav2Active()
            self.odom_init = False

    # def send_attach_request(self, rack_id):
    #     self.attacher = self.create_client(AttachLink,"/ATTACH_LINK")
    #     while not self.attacher.wait_for_service(timeout_sec=1.0):
    #          self.get_logger().info('Link attacher service not available, waiting again...')
    #     self.get_logger().info('Connected to Attach Server')

    #     req = AttachLink.Request()
    #     req.model1_name =  'ebot'     
    #     req.link1_name  = 'ebot_base_link'       
    #     req.model2_name =  rack_id     
    #     req.link2_name  = 'link'  
    #     self.future1 = self.attacher.call_async(req)
    #     rclpy.spin_until_future_complete(self,self.future1)

    # def send_detach_request(self, rack_id):
    #     self.detacher = self.create_client(DetachLink,"/DETACH_LINK")
    #     while not self.detacher.wait_for_service(timeout_sec=1.0):
    #          self.get_logger().info('Link attacher service not available, waiting again...')
    #     self.get_logger().info('Connected to Detach Server')
    #     req = DetachLink.Request()
    #     req.model1_name =  'ebot'     
    #     req.link1_name  = 'ebot_base_link'       
    #     req.model2_name =  rack_id      
    #     req.link2_name  = 'link'  
    #     self.future2 = self.detacher.call_async(req)
    #     rclpy.spin_until_future_complete(self,self.future2)
    
    def switch_eletromagent(self,relayState):
        self.get_logger().info('Changing state of the relay to '+str(relayState))
        self.trigger_usb_relay = self.create_client(RelaySw, 'usb_relay_sw')
        while not self.trigger_usb_relay.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('USB Trigger Service not available, waiting...')

        request_relay = RelaySw.Request()
        request_relay.relaychannel = True
        # request_relay.relaystate = relaySate
        request_relay.relaystate = relayState
        self.usb_relay_service_resp=self.trigger_usb_relay.call_async(request_relay)
        rclpy.spin_until_future_complete(self, self.usb_relay_service_resp)
        if(self.usb_relay_service_resp.result().success== True):
            self.get_logger().info(self.usb_relay_service_resp.result().message)
        else:
            self.get_logger().warn(self.usb_relay_service_resp.result().message)

    
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
    docker.switch_eletromagent(True)

    pose = [1.05, 2.02, 0.0]
    # pose = [6.3,-0.05, 3.14]
    goalPose1 = PoseStamped()
    goalPose1.header.frame_id = 'map'
    goalPose1.header.stamp = docker.navigator.get_clock().now().to_msg()
    goalPose1.pose.position.x = pose[0]
    goalPose1.pose.position.y = pose[1]
    q_1 = quaternion_from_euler(0,0,pose[2])
    goalPose1.pose.orientation.x = q_1[0]
    goalPose1.pose.orientation.y = q_1[1]
    goalPose1.pose.orientation.z = q_1[2]
    goalPose1.pose.orientation.w = q_1[3]
    
    docker.navigate(goalPose=goalPose1)
    docker.dock_request(True,True,9.0,0.0)
    sleep(3.0)


    # pose2 = [5.50, -0.05, 3.14]
    pose2 = [6.30,-0.05,3.14]

    goalPose2 = PoseStamped()
    goalPose2.header.frame_id = 'map'
    goalPose2.header.stamp = docker.navigator.get_clock().now().to_msg()
    goalPose2.pose.position.x = pose2[0]
    goalPose2.pose.position.y = pose2[1]
    q_1 = quaternion_from_euler(0,0,pose2[2])
    goalPose2.pose.orientation.x = q_1[0]
    goalPose2.pose.orientation.y = q_1[1]
    goalPose2.pose.orientation.z = q_1[2]
    goalPose2.pose.orientation.w = q_1[3]

    docker.navigate(goalPose2)

    sleep(3.0)

    docker.dock_request(False,True,0.0,3.14)

    docker.switch_eletromagent(False)
    docker.dock_request(True,True,50.0,3.14)


    sleep(2.0)
    docker.navigate(docker.initial_pose)

    docker.dock_request(False,True,0.1,0.0)


    docker.navigator.lifecycleShutdown()

    

    

    exit(0)


if __name__ == '__main__':
    main()
