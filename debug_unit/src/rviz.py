#!/usr/bin/env python
import numpy as np

import rospy
from tf2_ros import TransformBroadcaster
from tf_conversions import transformations
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from custom_msgs.msg import slam_debug_msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField


def publish_to_rviz(msg):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'

    # formula tf
    br = TransformBroadcaster()
    formula_tf = TransformStamped()
    formula_tf.header = header
    formula_tf.child_frame_id = 'formula'
    formula_tf.transform.translation.x = msg.pos_x
    formula_tf.transform.translation.y = msg.pos_y
    formula_tf.transform.translation.z = 0;
    q = transformations.quaternion_from_euler(0, 0, msg.theta) #roll-pith-yaw
    formula_tf.transform.rotation.x = q[0]
    formula_tf.transform.rotation.y = q[1]
    formula_tf.transform.rotation.z = q[2]
    formula_tf.transform.rotation.w = q[3]
    br.sendTransform(formula_tf)

    # cones points
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        # PointField('r', 12, PointField.UINT8, 1),
        # PointField('g', 16, PointField.UINT8, 1),
        # PointField('b', 20, PointField.UINT8, 1)
             ]
    # yellow cloud
    yellow_cloud = [[x,y,0] for x,y in zip(msg.yellow_cluster_x,msg.yellow_cluster_y)]
    yellow_cloud_msg = point_cloud2.create_cloud(header, fields, yellow_cloud)
    yellow_cloud_pub.publish(yellow_cloud_msg)
    # yellow cones
    yellow_cones = [[x,y,0] for x,y in zip(msg.yellow_cones_x,msg.yellow_cones_y)]
    yellow_cones_msg = point_cloud2.create_cloud(header, fields, yellow_cones)
    yellow_cones_pub.publish(yellow_cones_msg)
    # blue cloud
    blue_cloud = [[x,y,0] for x,y in zip(msg.blue_cluster_x,msg.blue_cluster_y)]
    blue_cloud_msg = point_cloud2.create_cloud(header, fields, blue_cloud)
    blue_cloud_pub.publish(blue_cloud_msg)
    # blue cones
    blue_cones = [[x,y,0] for x,y in zip(msg.blue_cones_x,msg.blue_cones_y)]
    blue_cones_msg = point_cloud2.create_cloud(header, fields, blue_cones)
    blue_cones_pub.publish(blue_cones_msg)
    # mid points
    mid_points = [[x,y,0] for x,y in zip(msg.mid_points_x,msg.mid_points_y)]
    mid_points_msg = point_cloud2.create_cloud(header, fields, mid_points)
    mid_points_pub.publish(mid_points_msg)


if __name__ == '__main__':
    rospy.init_node('debug_unit')
    topic_in = rospy.get_param('/cones_map/debug_topic', 'slam_debug')
    rospy.Subscriber(topic_in, slam_debug_msg , publish_to_rviz)
    # markers_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 100)
    yellow_cloud_pub = rospy.Publisher('yellow_cluster', PointCloud2, queue_size = 100)
    yellow_cones_pub = rospy.Publisher('yellow_cones', PointCloud2, queue_size = 100)
    blue_cloud_pub = rospy.Publisher('blue_cluster', PointCloud2, queue_size = 100)
    blue_cones_pub = rospy.Publisher('blue_cones', PointCloud2, queue_size = 100)
    rospy.spin()
   