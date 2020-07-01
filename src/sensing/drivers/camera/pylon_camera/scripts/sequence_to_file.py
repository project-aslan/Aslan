#! /usr/bin/env python

# __author__ = 'nikolas'

import os
import cv2

import rospy
from cv_bridge import CvBridge
import actionlib

from camera_control_msgs.msg import GrabSequenceAction, GrabSequenceGoal

server = None


def get_images(goal_folder):

    if not os.path.isdir(goal_folder):
        rospy.logerr("'"+goal_folder+"' is no directory")
        exit(1)

    client = actionlib.SimpleActionClient("/image_file_sequencer", GrabSequenceAction)
    client.wait_for_server()
    rospy.loginfo("Got server")

    goal = GrabSequenceGoal()
    goal.desiredExposureTimes = [40, 700, 7000]  # todo: select exposures as soon as supported

    client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    client.wait_for_result()
    result = client.get_result()

    if not result.success:
        rospy.logerr("Action returned but failed")
        exit(1)

    bridge = CvBridge()
    for i in range(len(result.exposureTimes)):
        file_name = goal_folder+'/'+"img_"+str(int(result.exposureTimes[i]))+".png"
        print file_name
        mat = bridge.imgmsg_to_cv2(result.images[i], "mono8")
        cv2.imwrite(file_name, mat)

    rospy.loginfo("Wrote " + str(len(result.exposureTimes)) + " images to " + goal_folder)


if __name__ == "__main__":
    rospy.init_node("image_sequence_to_file")

    get_images("/tmp")
    exit(0)
