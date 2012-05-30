#! /usr/bin/env python

import roslib; roslib.load_manifest('face_detector')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the face_detector action, including the
# goal message and the result message.
import face_detector.msg

def face_detector_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('face_detector_action', face_detector.msg.FaceDetectorAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = face_detector.msg.FaceDetectorGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('face_detector_action_client')
        result = face_detector_client()
        print "Done action"
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
