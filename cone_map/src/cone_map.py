#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from sklearn.cluster import DBSCAN
import numpy as np

def callBack(msg):
	np_arr = create_np_arr(msg)
	#first row is position so ignore
	#second row is amount of cones
	pose = np_arr[0]
	NRED = np_arr[1,0];
	NBLUE = np_arr[1,1];
	non_cones = 2
	if np_arr.shape[0] <= non_cones:
		return
	red_cones = np_arr[non_cones:non_cones+NRED]
	blue_cones = np_arr[non_cones+NRED:]
	if NRED:
		red_labels = DBSCAN(eps = 0.5, min_samples = 2).fit_predict(red_cones)

	if NBLUE:
		blue_labels = DBSCAN(eps = 0.5, min_samples = 2).fit_predict(blue_cones)

	# arr = create_Point_arr(points)
	# data.points = arr
	# pub.publish(data)

def create_Float32MultiArray(points):
	arr = []
	# something
	return arr

def create_np_arr(msg):
	data = msg.data
	layout = msg.layout
	dim0 = layout.dim[0]
	dim1 = layout.dim[1]
	rows = dim0.size
	cols = dim1.size
	arr = []
	for i in range(rows):
		row = []
		for j in range(cols):
			row += [data[i*cols+j]]
		arr += [row]
	return np.array(arr)

def listener():
	global pub
	rospy.init_node('listener', anonymous = True)
	topic_in = rospy.get_param('/cone_slam/points_topic', 'points_map')
	topic_out = rospy.get_param('/cone_slam/cone_topic', 'cone_map')
	pub = rospy.Publisher(topic_out, Float32MultiArray, queue_size = 10)
	rospy.Subscriber(topic_in, Float32MultiArray, callBack)
	rospy.spin()

if __name__ == '__main__':
	listener()