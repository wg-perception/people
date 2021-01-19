#! /usr/bin/env python

import actionlib

from face_detector.msg import FaceDetectorAction, FaceDetectorGoal

import rospy


def face_detector_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('face_detector_action', FaceDetectorAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server. (no parameters)
    goal = FaceDetectorGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()  # people_msgs/PositionMeasurement[] face_positions


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('face_detector_action_client')
        result = face_detector_client()
        print('Done action')
    except rospy.ROSInterruptException:
        print('Program interrupted before completion')
