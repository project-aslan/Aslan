#!/usr/bin/env python
# license removed for brevity

import rospy
import sensor_msgs.point_cloud2 as pc2


class MseConverter:
    def __init__(self):
        self.fields = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('Object_Number', 12, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('Speed', 16, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('Heading' ,         20, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('Quality', 24, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('Acceleration', 28, pc2.PointField.FLOAT32, 1)]
        # define publisher for the pointcloud with the cartesian coordinates
        self.pub = rospy.Publisher('object_list', pc2.PointCloud2, queue_size=1)
        # define subscriber to pointcloud
        rospy.Subscriber("radar_data", pc2.PointCloud2, self.pc_callback)    

    def pc_callback(self, data):
        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names=None))

        # Get the index of the needed fields
        for field, i in zip(data.fields, range(len(data.fields))):
            if field.name == "Object_Number":
                obj_idx = i
            if field.name =="x_Point":
                x_idx = i
            if field.name == "y_Point":
                y_idx = i
            if field.name == "Speed":
                speed_idx = i
            if field.name == "Heading":
                head_idx = i
            if field.name == "Quality":
                qual_idx = i
            if field.name == "Acceleration":
                acc_idx = i

        points = []
        for single_point in cloud_points:
            x = single_point[x_idx]
            y = single_point[y_idx]
            obj_num = single_point[obj_idx]
            speed = single_point[speed_idx]
            heading = single_point[head_idx]
            quality = single_point[qual_idx]
            acc = single_point[acc_idx]
            points.append([x, y, 0, obj_num, speed, heading, quality, acc])

        cloud_msg = pc2.create_cloud(data.header, self.fields, points)
        self.pub.publish(cloud_msg)


if __name__ == '__main__':
    rospy.init_node('target_list_2_MSE', anonymous=True)
    tl2cart = MseConverter()
    rospy.spin()
