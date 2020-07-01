#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from actionlib import SimpleActionClient, SimpleActionServer
from camera_control_msgs.msg import GrabImagesAction, GrabImagesGoal
from camera_control_msgs.msg import GrabAndSaveImageAction, GrabAndSaveImageResult


class GrabAndSaveImageActionServers():
    """
    This nodes provides action server that extend the 'grab_images_raw' and
    'grab_images_rect' action servers from the pylon_camera_node.
    The new action servers are named 'grab_and_save_imgage_raw' and
    'grab_and_save_imgage_rect'. They have basically the same action goal as
    the 'GrabImagesAction' in case of grabbing only one image, but they extend
    above goal with a sting describing the full storage path and name of the
    image to be saved
    """

    def __init__(self):
        camera_name = rospy.get_param('~camera_name', '')
        if not camera_name:
            rospy.logwarn("No camera name given! Assuming 'pylon_camera_node' as"
                          " camera name")
            camera_name = '/pylon_camera_node'
        else:
            rospy.loginfo('Camera name is: ' + camera_name)

        self._grab_imgs_raw_ac = SimpleActionClient('{}/grab_images_raw'.format(camera_name),
                                                    GrabImagesAction)

        self._grab_imgs_rect_ac = SimpleActionClient('{}/grab_images_rect'.format(camera_name),
                                                     GrabImagesAction)

        if self._grab_imgs_raw_ac.wait_for_server(rospy.Duration(10.0)):
            self._grab_and_save_img_raw_as = SimpleActionServer(
                "~grab_and_save_image_raw",
                GrabAndSaveImageAction,
                execute_cb=self.grab_and_save_img_raw_execute_cb,
                auto_start=False)
            self._grab_and_save_img_raw_as.start()
            rospy.loginfo('Found action server at '
                          '{}/grab_images_raw'.format(camera_name))
        else:
            rospy.logerr('Could not connect to action server at '
                         '{}/grab_images_raw'.format(camera_name))

        if self._grab_imgs_rect_ac.wait_for_server(rospy.Duration(2.0)):
            self._grab_and_save_img_rect_as = SimpleActionServer(
                "~grab_and_save_image_rect",
                GrabAndSaveImageAction,
                execute_cb=self.grab_and_save_img_rect_execute_cb,
                auto_start=False)
            self._grab_and_save_img_rect_as.start()
            rospy.loginfo('Found action server at '
                          '{}/grab_images_rect'.format(camera_name))
        else:
            rospy.logwarn('Could not connect to action server at '
                          '{}/grab_images_rect'.format(camera_name))

    def convert_goals(self, grab_and_save_img_goal, grab_imgs_goal):
        grab_imgs_goal.exposure_given = grab_and_save_img_goal.exposure_given
        if grab_and_save_img_goal.exposure_time:
            grab_imgs_goal.exposure_times.append(grab_and_save_img_goal.exposure_time)
        grab_imgs_goal.gain_given = grab_and_save_img_goal.gain_given
        if grab_and_save_img_goal.gain_value:
            grab_imgs_goal.gain_values.append(grab_and_save_img_goal.gain_value)
        grab_imgs_goal.gamma_given = grab_and_save_img_goal.gamma_given
        if grab_and_save_img_goal.gamma_value:
            grab_imgs_goal.gamma_values.append(grab_and_save_img_goal.gamma_value)
        grab_imgs_goal.brightness_given = grab_and_save_img_goal.brightness_given
        if grab_and_save_img_goal.brightness_value:
            grab_imgs_goal.brightness_values.append(grab_and_save_img_goal.brightness_value)
        grab_imgs_goal.exposure_auto = grab_and_save_img_goal.exposure_auto
        grab_imgs_goal.gain_auto = grab_and_save_img_goal.gain_auto

    def grab_and_save_img_raw_execute_cb(self, grab_and_save_img_goal):
        self.grab_and_save_img_execute_cb(grab_and_save_img_goal,
                                          self._grab_imgs_raw_ac,
                                          self._grab_and_save_img_raw_as)

    def grab_and_save_img_rect_execute_cb(self, grab_and_save_img_goal):
        self.grab_and_save_img_execute_cb(grab_and_save_img_goal,
                                          self._grab_imgs_rect_ac,
                                          self._grab_and_save_img_rect_as)

    def grab_and_save_img_execute_cb(self, grab_and_save_img_goal,
                                     action_client, action_server):
        grab_imgs_goal = GrabImagesGoal()
        self.convert_goals(grab_and_save_img_goal, grab_imgs_goal)

        action_client.send_goal(grab_imgs_goal)
        action_client.wait_for_result(rospy.Duration.from_sec(10.0))
        grab_imgs_result = action_client.get_result()

        grab_and_save_img_result = GrabAndSaveImageResult()

        if grab_imgs_result is not None and grab_imgs_result.success:
            filename = grab_and_save_img_goal.img_storage_path_and_name
            try:
                cv_img = CvBridge().imgmsg_to_cv2(grab_imgs_result.images[0],
                                                  desired_encoding='passthrough')
            except CvBridgeError as exception:
                rospy.logerr('Error converting img_msg_to_cv_img: ' +
                             str(exception))
                grab_and_save_img_result.success = False

            rospy.loginfo('Writing image to ' + filename)
            cv2.imwrite(filename, cv_img)
            grab_and_save_img_result.success = True
        else:
            grab_and_save_img_result.success = False
        action_server.set_succeeded(grab_and_save_img_result)

    def spin(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("grab_and_save_image_action_server")
    grab_and_save_img_as_node = GrabAndSaveImageActionServers()
    grab_and_save_img_as_node.spin()
