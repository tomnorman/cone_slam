#!/usr/bin/env python
import rospy
import std_msgs.geometry_msgs.Point as Point
import custom_msgs.slam_map as smap
from sklearn.cluster import DBSCAN
import numpy as np

rospy.init_node('listener', anonymous = True)
pub = rospy.Publisher("cones_map", smap, queue_size = 10)

def callBack(data):
	points = create_np_arr(data.points)
	mat = DBSCAN(eps = 0.5, min_samples = 2).fit_predict(points)
	arr = create_Point_arr(points)
	data.points = arr
	pub.publish(data)

def create_Point_arr(points):
	arr = []
	for po in points:
		arr = arr.append(Point(po[0],po[1],po[2]))
	return arr

def create_np_arr(points):
	arr = []
	for po in points:
		arr = arr.append([po.x, po.y, po.z])
	return arr

def listener():
	rospy.subscriber("points_map", smap, callBack)
	rospy.spin()

if __name__ == '__main__':
	listener()