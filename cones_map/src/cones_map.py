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

    pose = np.array([msg.pos_x, msg.pos_y, msg.pos_z])

    normal = np.array([msg.normal_x, msg.normal_y, msg.normal_z])

    yellow_cones = np.array([])
    blue_cones = np.array([])

    YCONES = 0
    BCONES = 0
    if NYELLOW:
        yellow_cones = create_centers(yellow_points, eps, min_samples)
        YCONES = yellow_cones.shape[0]
        if YCONES:
            yellow_cones = np.hstack((yellow_cones, np.full((YCONES, 1), yellow))) #x,y,color
    if NBLUE:
        blue_cones = create_centers(blue_points, eps, min_samples)
        BCONES = blue_cones.shape[0]
        if BCONES:
            blue_cones = np.hstack((blue_cones, np.full((BCONES, 1), blue))) #x,y,color
        
    test_msg = slam_in()
    test_msg.pos_x = pose[0]
    test_msg.pos_y = pose[1]
    test_msg.pos_z = pose[2]
    test_msg.normal_x = normal[0]
    test_msg.normal_y = normal[1]
    test_msg.normal_z = normal[2]
    test_msg.NYELLOW = YCONES
    test_msg.NBLUE = BCONES
    if YCONES:
        test_msg.yellow_x = yellow_cones[:,0].tolist()
        test_msg.yellow_y = yellow_cones[:,1].tolist()
    else:
        test_msg.yellow_x = [0]
        test_msg.yellow_y = [0]
    if BCONES:
        test_msg.blue_x = blue_cones[:,0].tolist()
        test_msg.blue_y = blue_cones[:,1].tolist()
    else:
        test_msg.blue_x = [0]
        test_msg.blue_y = [0]

    test_pub.publish(test_msg)
    
    #out_msg = path_array()
    #if NBLUE or NYELLOW:
        #mid_points = OrderCones(np.vstack((yellow_cones, blue_cones)), pose, normal)
        #out_msg.x = pose[0]
        #out_msg.y = pose[1]
        #out_msg.theta = np.arctan2(normal[1], normal[0])
        #out_msg.x_cones = mid_points[:,0].tolist()
        #out_msg.y_cones = mid_points[:,1].tolist()

    #pub.publish(out_msg)


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

def listener():
    global pub, test_pub
    rospy.init_node('listener', anonymous = True)
    # get points cluster from orbslam
    topic_in = rospy.get_param('/orb_slam2/points_topic', 'points_map')
    rospy.Subscriber(topic_in, slam_in, callBack)
    # topic to send output of Alon
    topic_out = rospy.get_param('/cones_map/cones_topic', 'cones_map')
    pub = rospy.Publisher(topic_out, path_array, queue_size = 100)
    # topic to send to plotter
    topic_test_out = rospy.get_param('/cones_map/test_topic', 'slam_test')
    test_pub = rospy.Publisher(topic_test_out, slam_in, queue_size = 100)

    rospy.spin()

if __name__ == '__main__':
    listener()
