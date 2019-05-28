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
    if counter %5 == 0:
        flag = 1-flag
        #plt.annotate("",xy=(normalx, normaly), xytext=(msg.pos_x, msg.pos_y), arrowprops=dict(arrowstyle="->"))
        plt.arrow(msg.pos_x, msg.pos_y, 0.06*msg.normal_x/normal_s, 0.06*msg.normal_y/normal_s)
        plt.scatter(msg.yellow_x, msg.yellow_y, color='yellow')
        plt.scatter(msg.blue_x, msg.blue_y, color='blue')
        if flag:
            color = 'red'
        else:
            color = 'pink'
        plt.plot([msg.pos_x], [msg.pos_y], marker='o', color=color)
        plt.xlim(-1.2, 1.2)
        plt.ylim(-1.2, 1.2)
        #plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

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
