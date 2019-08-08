#!/usr/bin/env python
import rospy

import tf_conversions

import tf2_ros
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import slam_test_msg

def publish_to_rviz(msg):
    br = tf2_ros.TransformBroadcaster()

    # yellow_cluster
    yellow_cluster = create_cloud(msg.yellow_cluster_x, msg.yellow_cluster_y, np.zeros(len(msg.yellow_cluster_y)),ColorRGBA())
    # blue_cluster
    blue_cluster= create_cloud(msg.blue_cluster_x, msg.blue_cluster_y, np.zeros(len(msg.blue_cluster_y)))
    # formula tf
    formula_tf = TransformStamped()
    formula_tf.header.stamp = rospy.Time.now()
    formula_tf.header.frame_id = 'world'
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
    header = Marker.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'

    points = [Point(x,y,z) for (x,y,z) in (xs,ys,zs)]
    marker = Marker()
    marker.header = header
    marker.type = 8
    marker.color = color
    marker.lifetime = 1
    #marker.frame_locked
    marker.points = points

    return points

if __name__ == '__main__':
    rospy.init_node('debug_unit')
    topic_in = rospy.get_param('/cones_map/test_topic', 'slam_test')
    rospy.Subscriber(topic_in, slam_test_msg , publish_to_rviz)
    markers_pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size = 1)
    rospy.spin()
   