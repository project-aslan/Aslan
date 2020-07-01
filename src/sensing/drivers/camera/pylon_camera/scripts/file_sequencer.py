#! /usr/bin/env python
#pylint: disable=E0611
import os
import cv2

import rospy
import actionlib
from cv_bridge import CvBridge

from camera_control_msgs.msg import GrabSequenceAction, GrabSequenceResult

__author__ = 'nikolas'

server = None
bridge = CvBridge()


def load_folder(folder):
    file_list = os.listdir(folder)
    print file_list
    file_map_ = dict()
    for f in file_list:
        try:
            s = int(f.split("_")[-1].split('.')[0])  # ???_number.??? -> number
        except ValueError as e:
            rospy.logerr("Invalid filename "+f+str(e))
            continue
        file_map_[s] = folder+"/"+f
    return file_map_


def select_images(file_map_, req_list):
    # select each image seperately (could lead to double images in result)
    res = GrabSequenceResult()

    res.exposureTimes = []

    res.images = []  #

    for t in sorted(file_map_.keys()):
        print t,
    print

    for t in req_list:
        print t,
        best_exp = min(file_map_.keys(), key=lambda x: abs(x-t))
        res.exposureTimes.append(best_exp)
        best_file = file_map_[best_exp]

        # create sensor_msgs/Image from files
        print best_file
        img = cv2.imread(best_file, 0)
        as_sensor_msg = bridge.cv2_to_imgmsg(img, "mono8")
        res.images.append(as_sensor_msg)
    res.success = True
    return res


def grab_sequence_callback(goal):
    folder = "/home/nikolas/Documents/sequence_test"
    # folder = rospy.get_param("~data_folder")
    if not os.path.isdir(folder):
        rospy.logerr("'"+folder+"' is no directory")
        res = GrabSequenceResult()
        res.success = False
        server.set_succeeded(res)
        return

    file_map = load_folder(folder)
    print file_map

    res = select_images(file_map, goal.desiredExposureTimes)

    server.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("image_file_sequencer")

    server = actionlib.SimpleActionServer("/image_file_sequencer", GrabSequenceAction,
                                          execute_cb=grab_sequence_callback, auto_start=False)
    server.start()
    rospy.spin()
