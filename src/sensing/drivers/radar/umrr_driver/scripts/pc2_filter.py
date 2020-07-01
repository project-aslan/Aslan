#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
import PyKDL
import sensor_msgs.point_cloud2 as pc2
from math import pi, sin
import numpy as np

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from umrr_driver.cfg import pc2filterConfig as ConfigType


class pc2_filter:

    def __init__(self):
        # Create a dynamic reconfigure server.
        self.reconf_server = dyn_reconfigure()
        # setup subscriber to spherical coordinates
        self.sub = rospy.Subscriber("radar_data", pc2.PointCloud2,self.mod_pcl, queue_size=1)
        # setup publisher for spherical coordinates
        self.pub = rospy.Publisher('filtered_data', pc2.PointCloud2, queue_size=1)

    def mod_pcl(self, cloud):
        # setup temporary point list
        points = []
        # read in all information from msg
        for field, i in zip(cloud.fields, range(len(cloud.fields))):
            if field.name == "Azimuth":
                azimuth_index = i
            if field.name == "Range":
                range_index = i
            if field.name == "Elevation":
                elevation_index = i
            if field.name == "Speed_Radial":
                sp_index = i
            if field.name == "RCS":
                rcs_index = i
            if field.name == "Cycle_Duration":
                cycled_dur_index = i
            if field.name == "Number_Of_Objects":
                num_object_index = i

        for point in pc2.read_points(cloud):
            # # check for each filter stage if enabled
            # apply selected filter value
            if self.reconf_server.filter_speed:
                if point[sp_index] < self.reconf_server.speed_min:
                    continue
                if point[sp_index] > self.reconf_server.speed_max:
                    continue
            if self.reconf_server.filter_rcs:
                if point[rcs_index] < self.reconf_server.rcs_min:
                    continue
                if point[rcs_index] > self.reconf_server.rcs_max:
                    continue
            if self.reconf_server.filter_range:
                if point[range_index] < self.reconf_server.range_min:
                    continue
                if point[range_index] > self.reconf_server.range_max:
                    continue
            if self.reconf_server.filter_azimuth:
                if point[azimuth_index] > self.reconf_server.angle_left:
                    continue
                # point lies to the right
                if point[azimuth_index] < self.reconf_server.angle_right:
                    continue
            if self.reconf_server.filter_elevation:
                if point[elevation_index] > self.reconf_server.angle_top:
                    continue
                if point[elevation_index] < self.reconf_server.angle_bottom:
                    continue
            points.append(point)

        # create cloud message
        cloud_msg = pc2.create_cloud(cloud.header, cloud.fields, points)
        # publish message
        self.pub.publish(cloud_msg)


class dyn_reconfigure:

    def __init__(self):
        # setup up dynamic reconfigure server
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)

    def reconfigure_cb(self, config, dummy):
        """Create a callback function for the dynamic reconfigure server."""
        # Fill in local variables with values received from dynamic reconfigure
        self.filter_height = config["filter_height"]
        self.filter_rcs = config["filter_rcs"]
        self.filter_speed = config["filter_speed"]
        self.filter_range = config["filter_range"]
        self.filter_azimuth = config["filter_azimuth"]
        self.filter_elevation = config["filter_elevation"]
        self.height_min = config["height_min"]
        self.height_max = config["height_max"]
        self.rcs_min = config["rcs_min"]
        self.rcs_max = config["rcs_max"]
        self.speed_min = config["speed_min"]
        self.speed_max = config["speed_max"]
        self.range_min = config["range_min"]
        self.range_max = config["range_max"]
        self.angle_left = config["FOV_left"]
        self.angle_right = config["FOV_right"]
        self.angle_top = config["FOV_top"]
        self.angle_bottom = config["FOV_bottom"]

        return config


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pointcloud_filter')
    # Go to class functions that do all the heavy lifting.
    try:
       pcl = pc2_filter()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
