#!/usr/bin/env python
import rospy

import tf_conversions

import tf2_ros
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import slam_test_msg

def publish_to_rviz(msg):
    br = tf2_ros.TransformBroadcaster()

    zeros = [0 for i in range(len(msg.yellow_cluster_y))]
    # yellow_cluster
    yellow_cluster = create_cloud(msg.yellow_cluster_x, msg.yellow_cluster_y,zeros,ColorRGBA(1,0,0,0))
    # blue_cluster
    blue_cluster= create_cloud(msg.blue_cluster_x, msg.blue_cluster_y,zeros,ColorRGBA(0,0,1,0))
    # formula tf
    formula_tf = TransformStamped()
    formula_tf.header.stamp = rospy.Time.now()
    formula_tf.header.frame_id = 'world'
    formula_tf.child_frame_id = 'formula'
    formula_tf.transform.translation.x = msg.pos_x
    formula_tf.transform.translation.y = msg.pos_y
    formula_tf.transform.translation.z = 0;
    formula_tf.transform.rotation.x = 0
    formula_tf.transform.rotation.y = 0
    formula_tf.transform.rotation.z = 0
    formula_tf.transform.rotation.w = 1

    markers_pub.publish(yellow_cluster)
    markers_pub.publish(blue_cluster)
    br.sendTransform(formula_tf)


def create_cloud(xs, ys, zs, color):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'

    marker = Marker()
    marker.header = header
    marker.ns = 'cluster'
    marker.type = marker.POINTS
    marker.scale = Vector3(100,100,100)
    marker.color = color
    marker.lifetime = rospy.Duration.from_sec(100.0) #one sec
    #marker.frame_locked
    points = [Point(x,y,z) for (x,y,z) in zip(xs,ys,zs)]
    marker.points = points

    return marker

if __name__ == '__main__':
    rospy.init_node('debug_unit')
    topic_in = rospy.get_param('/cones_map/test_topic', 'slam_test')
    rospy.Subscriber(topic_in, slam_test_msg , publish_to_rviz)
    markers_pub = rospy.Publisher("visualization_marker", Marker, queue_size = 100)
    rospy.spin()
   