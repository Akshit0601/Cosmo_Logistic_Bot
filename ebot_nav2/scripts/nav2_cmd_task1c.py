
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose_1.pose.position.x = 1.260008 - 1
    ap1 = [0.5, -2.455, 3.14]
    goal_pose_1.pose.position.x = 0.5
    # goal_pose_1.pose.position.y = 4.350000 + 0.25
    goal_pose_1.pose.position.y = -2.455
    # q_1 = quaternion_from_euler(0,0,3.00)
    q_1 = quaternion_from_euler(0,0,3.14)
    goal_pose_1.pose.orientation.x = q_1[0]
    goal_pose_1.pose.orientation.y = q_1[1]
    goal_pose_1.pose.orientation.z = q_1[2]
    goal_pose_1.pose.orientation.w = q_1[3]


    navigator.goToPose(goal_pose_1)

    i = 0
    while not navigator.isTaskComplete():

        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
        navigator.lifecycleShutdown()
    else:
        print('Goal has an invalid return status!')
    
    # goal_pose_2 = PoseStamped()
    # goal_pose_2.header.frame_id = 'map'
    # goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    # pose2 = [2.0, -7.0, -1.57]
    # goal_pose_2.pose.position.x = pose2[0]
    # goal_pose_2.pose.position.y = pose2[1]
    # q_2 = quaternion_from_euler(0,0,pose2[2])
    # goal_pose_2.pose.orientation.x = q_2[0]
    # goal_pose_2.pose.orientation.y = q_2[1]
    # goal_pose_2.pose.orientation.z = q_2[2]
    # goal_pose_2.pose.orientation.w = q_2[3]


    # navigator.goToPose(goal_pose_2)
    # i = 0
    # while not navigator.isTaskComplete():

    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    #     navigator.lifecycleShutdown()
    # else:
    #     print('Goal has an invalid return status!')


    # goal_pose_3 = PoseStamped()
    # goal_pose_3.header.frame_id = 'map'
    # goal_pose_3.header.stamp = navigator.get_clock().now().to_msg()
    # pose3 = [-3.0, 2.5, 1.57]
    # goal_pose_3.pose.position.x = pose3[0]
    # goal_pose_3.pose.position.y = pose3[1]
    # q_3 = quaternion_from_euler(0,0,pose3[2])
    # goal_pose_3.pose.orientation.x = q_3[0]
    # goal_pose_3.pose.orientation.y = q_3[1]
    # goal_pose_3.pose.orientation.z = q_3[2]
    # goal_pose_3.pose.orientation.w = q_3[3]
    

    # navigator.goToPose(goal_pose_3)
    # i = 0
    # while not navigator.isTaskComplete():

    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    #     navigator.lifecycleShutdown()
    # else:
    #     print('Goal has an invalid return status!')
    

    # goal_pose_4 = PoseStamped()
    # goal_pose_4.header.frame_id = 'map'
    # goal_pose_4.header.stamp = navigator.get_clock().now().to_msg()
    # pose3 =  [-1.0, -5.0, 1.57]
    # goal_pose_4.pose.position.x = pose3[0]
    # goal_pose_4.pose.position.y = pose3[1]
    # q_4 = quaternion_from_euler(0,0,pose3[2])
    # goal_pose_4.pose.orientation.x = q_4[0]
    # goal_pose_4.pose.orientation.y = q_4[1]
    # goal_pose_4.pose.orientation.z = q_4[2]
    # goal_pose_4.pose.orientation.w = q_4[3]
    # navigator.goToPose(goal_pose_4)

    
    

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()