#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def move_to_goal(x, y, z, w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    rospy.loginfo(f"Sending goal: x={x}, y={y}, z={z}, w={w}")
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        return True
    else:
        rospy.logwarn("Failed to reach goal.")
        return False

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client')

        # Điểm 1
        point1 = {'x': 17.92121234887096, 'y': -6.309403060785586, 'z': -0.09601126182800931, 'w': 0.9953802477456509}

        # Điểm 2
        point2 = {'x': 13.956226717724066, 'y': -6.8111826877055455, 'z': -0.7724243795632093, 'w': 0.6351067452455462}


        # # Điểm 3
        # point3 = {'x': 0.7796982682457894, 'y': 4.560168820121124, 'z': -0.061133427090111145, 'w': 0.9981296028533659}

        # # Điểm 4
        # point4 = {'x': 0.4103094940558054, 'y': -0.8585105497362909, 'z': -0.7628870398167298, 'w': 0.6465317969594901}

        while not rospy.is_shutdown():
            rospy.loginfo("Moving to Point 1")
            move_to_goal(point1['x'], point1['y'], point1['z'], point1['w'])

            rospy.sleep(3)  # Chờ một lúc trước khi di chuyển đến điểm 2

            rospy.loginfo("Moving to Point 2")
            move_to_goal(point2['x'], point2['y'], point2['z'], point2['w'])

            rospy.sleep(3)  # Chờ một lúc trước khi di chuyển đến điểm 3

            # rospy.loginfo("Moving to Point 3")
            # move_to_goal(point3['x'], point3['y'], point3['z'], point3['w'])

            # rospy.sleep(3)  # Chờ một lúc trước khi di chuyển đến điểm 4

            # rospy.loginfo("Moving to Point 4")
            # move_to_goal(point4['x'], point4['y'], point4['z'], point4['w'])

            # rospy.sleep(3)  # Chờ trước khi quay lại điểm 1

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
