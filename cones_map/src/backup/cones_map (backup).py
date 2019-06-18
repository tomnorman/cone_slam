#!/usr/bin/env python
import rospy
import numpy as np
from custom_msgs.msg import slam_in, path_array, slam_test_msg
from sklearn.cluster import DBSCAN
import OrderConesDT 
import PurePursuit
import serial


def callBack(msg):
    print calibration_flag
    calibration_flag = 1
    
    if calibration_flag:
        inp = raw_input()
        if 'calib' in inp:
            
    # DBSCAN consts
    eps = 0.05
    min_samples = 3

    yellow = 121 #number to identify cone
    blue = 98 #number to identify cone

    NYELLOW = msg.NYELLOW
    NBLUE = msg.NBLUE

    yellow_points = np.array([np.array(msg.yellow_x), np.array(msg.yellow_y)]).T
    blue_points = np.array([np.array(msg.blue_x), np.array(msg.blue_y)]).T

    pose2D = np.array([msg.pos_x, msg.pos_z])

    normal2D = np.array([msg.normal_x, msg.normal_z])
    theta = np.arctan2(normal2D[1], normal2D[0])

    yellow_cones = np.empty([1,3])
    blue_cones = np.empty([1,3])

    YCONES = 0
    BCONES = 0
    if NYELLOW:
        yellow_cones = create_centers(yellow_points, eps, min_samples)
        YCONES = yellow_cones.shape[0]
        if YCONES:
            yellow_cones = np.hstack((yellow_cones, np.full((YCONES, 1), yellow))) #x,y,color
    if NBLUE:
        print create_centers(blue_points, eps, min_samples)
        blue_cones = create_centers(blue_points, eps, min_samples)
        BCONES = blue_cones.shape[0]
        if BCONES:
            blue_cones = np.hstack((blue_cones, np.full((BCONES, 1), blue))) #x,y,color
        
    test_msg = slam_test_msg()
    test_msg.pos_x = pose2D[0]
    test_msg.pos_y = pose2D[1]
    test_msg.normal_x = normal2D[0]
    test_msg.normal_y = normal2D[1]
    test_msg.theta = theta
    if NYELLOW:
        test_msg.yellow_cluster_x = yellow_points[:,0].tolist()
        test_msg.yellow_cluster_y = yellow_points[:,1].tolist()
    if NBLUE:
        test_msg.blue_cluster_x = blue_points[:,0].tolist()
        test_msg.blue_cluster_y = blue_points[:,1].tolist()
    if YCONES:
        test_msg.yellow_cones_x = yellow_cones[:,0].tolist()
        test_msg.yellow_cones_y = yellow_cones[:,1].tolist()
    if BCONES:
        test_msg.blue_cones_x = blue_cones[:,0].tolist()
        test_msg.blue_cones_y = blue_cones[:,1].tolist()

    print yellow_cones.shape, blue_cones.shape
    Cones=np.vstack((yellow_cones, blue_cones))
    if Cones.shape[0] > 0:
        car_pos=np.array([pose2D[0],pose2D[1]]) 
        ConesR=Cones[:,:2]-car_pos #vectors of cones relative to car
        SqDistance=np.diag(np.matmul(ConesR,np.transpose(ConesR)))
        minR=(min(SqDistance))**0.5
        
        MapTrack = OrderConesDT.MapTrack(Cones, pose2D, normal2D,StandardDistance=minR*5,SphereRFactor=1,CarLengthFactor=0.1)  # Build class

        MapTrack.OrderCones(MaxItrAmnt=20, CostThreshold=-0.2, ColorCostWeight=0.4, \
               RRatioThreshold=10)

        mid_points = MapTrack.FindMidPoints()

        if mid_points.size>0:
            PP=PurePursuit.PPAckerman(pose2D,normal2D,mid_points,Lb=0.1,SymSteeringAngleBounds=14*3.1415/180,SphereRFactor=1,StandardDistance=1)
            print(round(24-PP.SteeringAngle*180/np.pi))
            

            #s = str(round(24-PP.SteeringAngle*180/np.pi))
            #if s>48 : 
            #    s=48
            #if s<0: 
            #    s=0

        #ard.write(chr(s))


        print 'test stuff' , mid_points
        if mid_points is None:
            test_pub.publish(test_msg)
            print 'no mid_points'
            return

        test_msg.mid_points_x = mid_points[:,0].tolist()
        test_msg.mid_points_y = mid_points[:,1].tolist()
        test_pub.publish(test_msg)

        # msg to pure_pursuit
        out_msg = path_array()
        out_msg.x = pose2D[0]
        out_msg.y = pose2D[1]
        out_msg.theta = 24*np.pi/180 -PP.SteeringAngle
        out_msg.x_cones = mid_points[:,0].tolist()
        out_msg.y_cones = mid_points[:,1].tolist()
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

def listener():
    global pub, test_pub
    rospy.init_node('listener', anonymous = True)
    # get points cluster from orbslam
    topic_in = rospy.get_param('/orb_slam2/points_topic', 'points_map')
    rospy.Subscriber(topic_in, slam_in, callBack)
    # topic to send output of Alon
    topic_out = rospy.get_param('/cones_map/cones_topic', 'carPoseForPurePursuit')
    pub = rospy.Publisher(topic_out, path_array, queue_size = 100)
    # topic to send to plotter
    topic_test_out = rospy.get_param('/cones_map/test_topic', 'slam_test')
    test_pub = rospy.Publisher(topic_test_out, slam_test_msg, queue_size = 100)

    rospy.spin()

if __name__ == '__main__':
    global calibration_flag
    calibration_flag = 0
    #port = '/dev/ttyACM0'
    #ard = "global"
    #ard = serial.Serial(port,9600,timeout=5)
    listener()
