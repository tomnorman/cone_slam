#! /usr/bin/python
import numpy as np
from matplotlib import pyplot as plt
import rospy



def plot_points():
	# update arrays and position from msg
	# what is the value of norm?
	global counter
	x1 = [0.52, 0.63, 0.733, 0.834, 0.94, 0.967, 0.98]
	y1 = [0.25, 0.43, 0.433, 0.534, 0.54, 0.767, 0.8]
	x2 = [0.1, 0.13, 0.233, 0.3, 0.34, 0.467, 0.58]
	y2 = [0.15, 0.23, 0.33, 0.4, 0.5, 0.5767, 0.68]
	posex = 0.35
	posey = 0.3
	#
	if counter %10 == 0:
		plt.annotate("",xy=(0.5,0.45), xytext=(0.35,0.3), arrowprops=dict(arrowstyle="->"))
		plt.scatter(x1, y1, color='yellow')
		plt.scatter(x2, y2, color='blue')
		plt.plot([posex], [posey], marker='*', color='red')
		plt.axis("equal")
		plt.draw()
		plt.pause(0.00000000001)

	counter += 1

if __name__ == '__main__':
	counter = 0
	rospy.init_node("plotter")
	#rospy.Subscriber("cone_map", ? , plot_points)
	plot_points()
	plt.ion()
	plt.show()
	rospy.spin()	
