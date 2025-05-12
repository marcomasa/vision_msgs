#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from math import pi, sin, cos
from numpy import array
from numpy.linalg import norm
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DWithPcl
from vision_msgs.msg import BoundingBox3D
from vision_msgs.msg import ObjectHypothesisWithPose

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from sensor_msgs_py.point_cloud2 import create_cloud


def quaternion_about_axis(angle, axis):
    axis = array(axis)
    axis = axis / norm(axis)
    half_angle = angle / 2
    sine = sin(half_angle)
    w = cos(half_angle)
    x, y, z = axis * sine
    return x, y, z, w


class pub_detection3_d_array(Node):
    def __init__(self):
        super().__init__("pub_detection3_d_with_pcl_sample")
        self.__pub = self.create_publisher(
            Detection3DWithPcl, "detection3_d_with_pcl", 10)
        self.__timer = self.create_timer(0.1, self.pub_sample)
        self.__counter = 0
        self.__header = Header()
        self.__msg_def = {
            "score": [0.0, 1.0, 2.0, 3.0, 1.0],
            "obj_id": ["", "", "car", "cyclist", "tree"]
        }
    

    def create_colored_pointcloud(self, center_x, center_y, center_z, num_points=100, spread_x=1.0, spread_y=1.0, spread_z=1.0, color=(255, 255, 255)) -> PointCloud2:
        import std_msgs.msg
        import struct  # For packing RGB values into a single integer

        # Generate random points around the center with individual spreads
        points = [
            [
                center_x + random.uniform(-spread_x, spread_x),
                center_y + random.uniform(-spread_y, spread_y),
                center_z + random.uniform(-spread_z, spread_z),
                struct.unpack('I', struct.pack('BBBB', color[2], color[1], color[0], 0))[0]  # Pack RGB into a single integer
            ]
            for _ in range(num_points)
        ]

        # Define fields for PointCloud2 with packed RGB
        from sensor_msgs.msg import PointField

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),  # Packed RGB field
        ]

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        pointcloud_msg = create_cloud(header, fields, points)

        return pointcloud_msg
    
    def create_random_pointcloud_with_spread(self, center_x, center_y, center_z, num_points=100, spread_x=1.0, spread_y=1.0, spread_z=1.0) -> PointCloud2:
        import std_msgs.msg

        # Generate random points around the center with individual spreads
        points = [
            [
                center_x + random.uniform(-spread_x, spread_x),
                center_y + random.uniform(-spread_y, spread_y),
                center_z + random.uniform(-spread_z, spread_z),
            ]
            for _ in range(num_points)
        ]

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        pointcloud_msg = create_cloud_xyz32(header, points)

        return pointcloud_msg

    def create_msg(self, bbox: BoundingBox3D, scores, obj_ids) -> Detection3DWithPcl:
        msg = Detection3DWithPcl()
        msg.header = self.__header
        msg.bbox = bbox
        for score, obj_id in zip(scores, obj_ids):
            obj = ObjectHypothesisWithPose()
            obj.hypothesis.score = score
            obj.hypothesis.class_id = obj_id
            msg.results.append(obj)
        
        
        #msg.source_cloud = self.create_random_pointcloud_with_spread(
        #    bbox.center.position.x,
        #    bbox.center.position.y,
        #    bbox.center.position.z,
        #    num_points=100,
        #    spread_x=bbox.size.x / 2.0,
        #    spread_y=bbox.size.y / 2.0,
        #    spread_z=bbox.size.z / 2.0,
        #)

        msg.source_cloud = self.create_colored_pointcloud(
            bbox.center.position.x,
            bbox.center.position.y,
            bbox.center.position.z,
            num_points=100,
            spread_x=bbox.size.x / 2.0,
            spread_y=bbox.size.y / 2.0,
            spread_z=bbox.size.z / 2.0,
            color=(255, 0, 0)  # Red color
        )

        return msg

    def pub_sample(self):
        while self.__pub.get_subscription_count() == 0:
            return
        self.__header.stamp = self.get_clock().now().to_msg()
        self.__header.frame_id = "map"

        # Reset counter and indices if counter is a multiple of 30
        if self.__counter % 10 == 0:
            self.__counter = 0

        # Get the current score and object ID
        score = random.sample(self.__msg_def["score"], 1)[0]
        obj_id = random.sample(self.__msg_def["obj_id"], 1)[0]

        # Create a single Detection3D message
        bbox = BoundingBox3D()
        #quat = quaternion_about_axis((self.__counter % 100) * pi * 2 / 100.0, [0, 0, 1])
        quat = quaternion_about_axis( 0 * pi * 2 / 100.0, [0, 0, 1])

        bbox.center.orientation.x = quat[0]
        bbox.center.orientation.y = quat[1]
        bbox.center.orientation.z = quat[2]
        bbox.center.orientation.w = quat[3]
        # Set the center position to a fixed value
        bbox.center.position.x = 1.5
        bbox.center.position.y = 0.0
        bbox.size.x = (self.__counter % 10 + 1) * 0.1
        bbox.size.y = ((self.__counter + 1) % 5 + 1) * 0.1
        bbox.size.z = ((self.__counter + 2) % 10 + 1) * 0.1
        detection_msg = self.create_msg(
            bbox=bbox, scores=[score], obj_ids=[obj_id]
        )

        # Publish the Detection3D message
        self.__pub.publish(detection_msg)
        self.__counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = pub_detection3_d_array()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
