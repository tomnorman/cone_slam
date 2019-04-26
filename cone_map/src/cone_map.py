#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import DBSCAN
import numpy as np

def callBack(data):
	points = create_np_arr(data.points)
	mat = DBSCAN(eps = 0.5, min_samples = 2).fit_predict(points)
	arr = create_Point_arr(points)
	data.points = arr
	pub.publish(data)

def create_Float32MultiArray(points):
	arr = []
	# something
	return arr

def create_np_arr(points):
	arr = []
	for po in points:
		arr = arr.append([po.x, po.y, po.z])
	return arr

def listener():
	global pub
	rospy.init_node('listener', anonymous = True)
	name = rospy.get_name()
	topic_in = rospy.get_param(name + '/points_topic', 'points_map1')
	topic_out = rospy.get_param(name + 'cone_topic', 'cone_map1')
	rospy.Subscriber(topic_in, Float32MultiArray, callBack)
	pub = rospy.Publisher(topic_out, Float32MultiArray, queue_size = 10)
	rospy.spin()

if __name__ == '__main__':
	listener()