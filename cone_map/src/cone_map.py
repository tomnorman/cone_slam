#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from custom_msgs.msg import slam_msg as smsg
from sklearn.cluster import DBSCAN
import numpy as np

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
	global pub
	rospy.init_node('listener', anonymous = True)
	rospy.subscriber("points_map", smsg, callBack)
	pub = rospy.Publisher("cones_map", smsg, queue_size = 10)
	rospy.spin()

if __name__ == '__main__':
	listener()