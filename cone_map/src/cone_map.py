#!/usr/bin/env python
import rospy
from custom_msgs.msg import slam_in, path_array
from sklearn.cluster import DBSCAN
import numpy as np
from OrderConesDT import OrderCones


def callBack(msg):
    # DBSCAN consts
    eps = 0.02
    min_samples = 3

    yellow = 121 #number to identify cone
    blue = 98 #number to identify cone

    NYELLOW = msg.NYELLOW
    NBLUE = msg.NBLUE

    yellow_points = np.array([np.array(msg.yellow_x), np.array(msg.yellow_y)]).T
    blue_points = np.array([np.array(msg.blue_x), np.array(msg.blue_y)]).T

    pose = np.array([msg.pos_x, msg.pos_y])

    normal = np.array([msg.normal_x, msg.normal_y])

    yellow_cones = np.array([])
    blue_cones = np.array([])
    if NYELLOW:
        yellow_cones = create_centers(yellow_points, eps, min_samples)
        YCONES = yellow_cones.shape(0)
        yellow_cones = np.hstack((yellow_cones, np.full((YCONES, 1), yellow))) #x,y,color
    if NBLUE:
        blue_cones = create_centers(blue_points, eps, min_samples)
        BCONES = blue_cones.shape(0)
        blue_cones = np.hstack((blue_cones, np.full((BCONES, 1), blue))) #x,y,color
    out_msg = path_array()
    if NBLUE || NYELLOW:
        mid_points = OrderCones(np.vstack((yellow_cones, blue_cones)), pose, normal)
        out_msg.x = pose[0]
        out_msg.y = pose[1]
        out_msg.theta = np.arctan2(normal[1], normal[0])
        out_msg.x_cones = mid_points[:,0]
        out_msg.y_cones = mid_points[:,1]

    pub.publish(out_msg)


def create_centers(samples, eps, min_samples):
    clusters = DBSCAN(eps = eps, min_samples = min_samples).fit_predict(samples)
    sorted_clusters_idxs = np.argsort(clusters)
    clusters = clusters[sorted_clusters_idxs]
    samples = samples[sorted_clusters_idxs] #cluster samples
    '''
    cone -1 #-1 is ouliers
    cone -1 #-1 is ouliers
    ...
    cone 0
    cone 0
    ...
    cone i
    cone i
    cone i
    ...
    cone n
    cone n
    '''
    centers = []
    for i in range(clusters[-1]+1): #clusters[-1] == np.max(clusters)
        centers += [np.mean(samples[clusters == i],axis = 0)] #get the mean of the cluster
        #TODO: geometric mean??
    return np.array(centers)


def create_np_arr(msg):
    data = msg.data
    layout = msg.layout
    dim0 = layout.dim[0]
    dim1 = layout.dim[1]
    rows = dim0.size
    cols = dim1.stride
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
    topic_in = rospy.get_param('/orb_slam2/points_topic', 'points_map')
    topic_out = 'cone_map'
    #pub = rospy.Publisher(topic_out, slam_in, queue_size = 100)
    rospy.Subscriber(topic_in, slam_in, callBack)

    rospy.spin()

if __name__ == '__main__':
    listener()
