#!/usr/bin/env python
import rospy

import tf_conversions
import numpy as np
import tf2_ros
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import slam_test_msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField


def publish_to_rviz(msg):
    br = tf2_ros.TransformBroadcaster()

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.UINT8, 1),
        PointField('g', 16, PointField.UINT8, 1),
        PointField('b', 20, PointField.UINT8, 1)
             ]
    # formula tf
    formula_tf = TransformStamped()
    formula_tf.header = header
    formula_tf.child_frame_id = 'formula'
    formula_tf.transform.translation.x = msg.pos_x
    formula_tf.transform.translation.y = msg.pos_y
    formula_tf.transform.translation.z = 0;
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    formula_tf.transform.rotation.x = q[0]
    formula_tf.transform.rotation.y = q[1]
    formula_tf.transform.rotation.z = q[2]
    formula_tf.transform.rotation.w = q[3]

    br.sendTransform(formula_tf)

    # yellow cloud
    yellow_cloud  = [[x,y,0,0,255,255] for x,y in zip(msg.yellow_cluster_x,msg.yellow_cluster_y)]
    yellow_cloud_msg = point_cloud2.create_cloud(header, fields, yellow_cloud)
    blue_cloud  = [[x,y,0,0,0,255] for x,y in zip(msg.blue_cluster_x,msg.blue_cluster_y)]
    blue_cloud_msg = point_cloud2.create_cloud(header, fields, blue_cloud)
    
    yellow_cloud_pub.publish(yellow_cloud_msg)
    blue_cloud_pub.publish(blue_cloud_msg)

    # marker_array = MarkerArray([yellow_cluster, blue_cluster])
    # markers_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('debug_unit')
    topic_in = rospy.get_param('/cones_map/test_topic', 'slam_test')
    rospy.Subscriber(topic_in, slam_test_msg , publish_to_rviz)
    # markers_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 100)
    yellow_cloud_pub = rospy.Publisher('yellow_cluster', PointCloud2, queue_size = 100)
    blue_cloud_pub = rospy.Publisher('blue_cluster', PointCloud2, queue_size = 100)
    rospy.spin()
   