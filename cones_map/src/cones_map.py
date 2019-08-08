#!/usr/bin/env python
import rospy
import numpy as np
from custom_msgs.msg import slam_in, path_array, slam_test_msg
from sklearn.cluster import DBSCAN
import OrderConesDT 
import PurePursuit
import serial

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

class Server:
    def __init__(self):
        self.calibrated = -1
        self.normalize_factor = 1
        
    def callBack(self,msg):
#        self.normalize_factor = 1
        # DBSCAN consts
        eps = 0.5
        min_samples = 3

        yellow = 121 #number to identify cone
        blue = 98 #number to identify cone

        NYELLOW = msg.NYELLOW
        NBLUE = msg.NBLUE

        yellow_points = np.array([np.array(msg.yellow_x), np.array(msg.yellow_y)]).T
        blue_points = np.array([np.array(msg.blue_x), np.array(msg.blue_y)]).T
        
        yellow_points = yellow_points*self.normalize_factor
        blue_points = blue_points*self.normalize_factor

        pose2D = np.array([msg.pos_x, msg.pos_z])
        pose2D = pose2D*self.normalize_factor

        normal2D = np.array([msg.normal_x, msg.normal_z])
        normal2D = normal2D*self.normalize_factor
        
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
            #print create_centers(blue_points, eps, min_samples)
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

        #print yellow_cones.shape, blue_cones.shape
        Cones = np.array([])
        if yellow_cones.size and blue_cones.size:
            Cones=np.vstack((yellow_cones, blue_cones))
        if Cones.shape[0] > 0:
            test_pub.publish(test_msg)
            car_pos=np.array([pose2D[0],pose2D[1]]) 
            ConesR=Cones[:,:2]-car_pos #vectors of cones relative to car
            SqDistance=np.diag(np.matmul(ConesR,np.transpose(ConesR)))
            minR=(min(SqDistance))**0.5
            
            MapTrack = OrderConesDT.MapTrack(Cones, pose2D, normal2D,SphereR=10,CarLength=1.6)  # Build class               

    
            MapTrack.OrderCones(MaxItrAmnt=20, CostThreshold=-0.2, ColorCostWeight=0.4, \
                   RRatioThreshold=10)

            mid_points = MapTrack.FindMidPoints()

            #Lb is car length
            PP=PurePursuit.PPAckerman(pose2D,normal2D,mid_points,Lb=1.6,SymSteeringAngleBounds=12*3.1415/180,SphereR=10)         

            #print(round(24-PP.SteeringAngle*180/np.pi))
                

            if mid_points is None:
                mid_points = np.zeros((2,2))

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
            

    def calibration(self,msg):
        if self.calibrated==-1:
            print 'first_input'
            inp = raw_input()
            if 'calib' in inp:
                print('stage1')
                self.pose_x1 = msg.pos_x
                self.pose_y1 = msg.pos_z
                self.calibrated = 0
                return
    
        if self.calibrated==0:
            print 'second_input'
            inp = raw_input()
            if 'calib' in inp:
                self.pose_x2 = msg.pos_x
                self.pose_y2 = msg.pos_z
                self.normalize_factor = 1.6 / np.sqrt((self.pose_y1-self.pose_y2)**2 + (self.pose_x1-self.pose_x2)**2)
        
                print self.normalize_factor
                self.calibrated=1



    


if __name__ == '__main__':

    rospy.init_node('listener', anonymous = True)

    server = Server()

    # get points cluster from orbslam
    topic_in = rospy.get_param('/orb_slam2/points_topic', 'points_map')
    rospy.Subscriber(topic_in, slam_in, server.callBack)
    
    rospy.Subscriber(topic_in, slam_in, server.calibration, queue_size = 1)
    
    
    
    # topic to send output of Alon
    topic_out = rospy.get_param('/cones_map/cones_topic', 'carPoseForPurePursuit')
    pub = rospy.Publisher(topic_out, path_array, queue_size = 100)
    # topic to send to plotter
    topic_test_out = rospy.get_param('/cones_map/test_topic', 'slam_test')
    test_pub = rospy.Publisher(topic_test_out, slam_test_msg, queue_size = 100)

    rospy.spin()
