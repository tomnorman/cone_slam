#!/usr/bin/env python
import numpy as np
import rospy
from matplotlib import pyplot as plt
from custom_msgs.msg import slam_in

def plot_slam(msg):
    # update arrays and position from msg
    # what is the value of norm?
    global counter, flag
    normal_s = np.sqrt(msg.normal_y**2 + msg.normal_x**2)
    plt.figure
    if counter %5 == 0:

        flag = 1-flag
        plt.cla()
        plt.arrow(msg.pos_x, msg.pos_y, 0.04*msg.normal_x/normal_s, 0.04*msg.normal_y/normal_s)
        plt.scatter(msg.yellow_x, msg.yellow_y, color='darkorange',s=10)
        plt.scatter(msg.blue_x, msg.blue_y, color='blue',s=10)

        plt.scatter(msg.mid_points_x,msg.mid_points_y,color=[0.5,0,0],s=5,edgecolors=[0,0,0]) #plot cones
        plt.plot(msg.mid_points_x,msg.mid_points_y,color=[0.5,0,0],lw=2) #plot interpolation

        if flag:
            color = 'darkorange'
        else:
            color = 'pink'
        #plt.plot([msg.pos_x], [msg.pos_y], marker='o', color=color)
        #plt.xlim(-2, 2)
        #plt.ylim(-0.5, 3.5)
        plt.axis("equal")
        plt.text(1,1,msg.theta)
        plt.show()
        plt.pause(0.1)

    counter += 1

if __name__ == '__main__':
    counter = 0
    flag = 0
    rospy.init_node('plotter', anonymous = True)
    topic_in = rospy.get_param('/cones_map/test_topic', 'slam_test')
    rospy.Subscriber(topic_in, slam_in , plot_slam)
    plt.ion()
    plt.show()
    rospy.spin()    
