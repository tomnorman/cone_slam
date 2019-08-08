#!/usr/bin/env python
import numpy as np
import rospy
from matplotlib import pyplot as plt
from custom_msgs.msg import slam_test_msg

arrow_scale = 0.01

def plot_slam(msg):
    global counter, arrow_scale
    arrow_size = np.sqrt(msg.normal_y**2 + msg.normal_x**2)
    plt.figure
    if counter %5 == 0:
        plt.cla()
        #plot normal
        plt.arrow(msg.pos_x, msg.pos_y, msg.normal_x/arrow_size, msg.normal_y/arrow_size)
        #plot cones and clusters
        plt.scatter(msg.yellow_cluster_x, msg.yellow_cluster_y, color = 'darkorange', s=3)
        plt.scatter(msg.yellow_cones_x, msg.yellow_cones_y, color='red',s=7)
        plt.scatter(msg.blue_cluster_x, msg.blue_cluster_y, color = 'aqua', s=3)
        plt.scatter(msg.blue_cones_x, msg.blue_cones_y, color='blue',s=7)
        #plot middle path
        if len(msg.mid_points_x) and len(msg.mid_points_y):
            plt.scatter(msg.mid_points_x,msg.mid_points_y,color=[0.5,0,0],s=5,edgecolors=[0,0,0]) #plot cones
            plt.plot(msg.mid_points_x,msg.mid_points_y,color=[0.5,0,0],lw=2) #plot interpolation

        #plt.plot([msg.pos_x], [msg.pos_y], marker='o', color=color)
        plt.axis("equal")
#        plt.xlim(msg.pos_x-1,msg.pos_x+1)
#        plt.ylim(msg.pos_y-1,msg.pos_y+1)
        plt.text(1,1,msg.theta)
        plt.show()
        plt.pause(0.1)

    counter += 1

if __name__ == '__main__':
    counter = 0
    rospy.init_node('plotter', anonymous = True)
    topic_in = rospy.get_param('/cones_map/test_topic', 'slam_test')
    rospy.Subscriber(topic_in, slam_test_msg , plot_slam)
    plt.ion()
    plt.show()
    rospy.spin()    
