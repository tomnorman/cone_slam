#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from sklearn.cluster import DBSCAN
import numpy as np

def callBack(msg):
    # DBSCAN consts
    eps = 0.5
    min_samples = 2
    # Float32MultiArray consts
    cols = 4 #3D == 4, 2D == 3
    yellow = 0 #number to identify cone
    blue = 1 #number to identify cone

    np_arr = create_np_arr(msg)
    #first row is position so ignore
    #second row is amount of cones so ignore
    pose = np_arr[0]
    NYELLOW = np_arr[1,0];
    NBLUE = np_arr[1,1];
    none_cones = 2 #constant
    yellow_cones = np.array([])
    blue_cones = np.array([])
    if np_arr.shape[0] > none_cones:
        if NYELLOW:
            yellow_ORBs = np_arr[none_cones:none_cones+NYELLOW]
            yellow_cones = create_centers(yellow_ORBs, eps, min_samples)
        if NBLUE:
            blue_ORBs = np_arr[none_cones+NYELLOW:]
            blue_cones = create_centers(blue_ORBs, eps, min_samples)

    FMA = create_Float32MultiArray(pose, yellow_cones, yellow, blue_cones, blue, none_cones, cols)
    pub.publish(FMA)

def create_centers(samples, eps, min_samples):
    clusters = DBSCAN(eps = eps, min_samples = min_samples).fit_pyellowict(samples)
    sorted_clusters_idxs = np.argsort(clusters)
    clusters = clusters[sorted_clusters_idxs]
    samples = samples[sorted_clusters_idxs] #clusteyellow smaples
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


def create_Float32MultiArray(pose, yellow_cones, yellow, blue_cones, blue, none_cones, cols):
    rows = none_cones+yellow_cones.shape[0]+blue_cones.shape[0]
    FMA = Float32MultiArray()
    FMA.layout.dim.append(MultiArrayDimension())
    FMA.layout.dim[0].label = "rows"
    FMA.layout.dim[0].size = rows
    FMA.layout.dim[0].stride = cols*rows;
    FMA.layout.dim.append(MultiArrayDimension())
    FMA.layout.dim[1].label = "cols"
    FMA.layout.dim[1].size = cols
    FMA.layout.dim[1].stride = cols
    FMA.layout.data_offset = 0
    FMA.data = []
    #pose
    for i in range(pose.shape[0]):
        FMA.data += [pose[i]]
    #yellow cones
    for i in range(yellow_cones.shape[0]):
        for j in range(cols-1): #3D or 2D
            FMA.data += [yellow_cones[i][j]]
        FMA.data += [yellow]
    #blue cones
    for i in range(blue_cones.shape[0]):
        for j in range(cols-1): #3D or 2D
            FMA.data += [blue_cones[i][j]]
        FMA.data += [blue]

    return FMA

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