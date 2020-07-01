#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from camera_control_msgs.msg import GrabImagesAction, GrabImagesGoal
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

class TriggeredImageTopic():
    """
    This nodes provides allows to transform a trigger topic to an image topic using GrabImageAction.
    Whenever a signal is published on ~trigger a sensor_msg.msg.Image is published on the output topic.
    
    The interface is restricted to a rosparam for exposure with a low gain
    """

    def __init__(self):
        self.camera_name = rospy.get_param('~camera_name', '')
        self.output_topic_name = rospy.get_param('~triggered_image_topic', 'triggered_images')
        if not self.camera_name:
            rospy.logwarn("No camera name given! Assuming 'pylon_camera_node' as"
                          " camera name")
            self.camera_name = '/pylon_camera_node'
        else:
            rospy.loginfo('Camera name is: ' + self.camera_name)

        self._grab_imgs_rect_ac = SimpleActionClient('{}/grab_images_rect'.format(self.camera_name),
                                                     GrabImagesAction)

        if self._grab_imgs_rect_ac.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo('Found action server at '
                          '{}/grab_images_raw'.format(self.camera_name))
        else:
            rospy.logerr('Could not connect to action server at '
                         '{}/grab_images_raw'.format(self.camera_name))
        self.pub = rospy.Publisher(self.output_topic_name, Image, queue_size=10)
        self.subscriber = rospy.Subscriber("~trigger", Empty, self.trigger_cb)

    def trigger_cb(self, msg):
        grab_imgs_goal = GrabImagesGoal()
        
        
        grab_imgs_goal.exposure_given = True
        grab_imgs_goal.exposure_times = [rospy.get_param('~exposure_time', 20000.0)]
        grab_imgs_goal.gain_given = True
        grab_imgs_goal.gain_values = [0.2]
        grab_imgs_goal.gain_auto = False
         
        self._grab_imgs_rect_ac.send_goal(grab_imgs_goal)
        self._grab_imgs_rect_ac.wait_for_result(rospy.Duration.from_sec(10.0))
        grab_imgs_result = self._grab_imgs_rect_ac.get_result()
        rospy.loginfo("Got image")
        if len(grab_imgs_result.images) > 0:
            rospy.loginfo("publish image")
            self.pub.publish(grab_imgs_result.images[0])


    def spin(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("triggered_img_topic")
    triggered_img_topic = TriggeredImageTopic()
    triggered_img_topic.spin()
