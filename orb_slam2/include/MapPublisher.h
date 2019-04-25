#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <vector>
#include <iostream>

#include "MapPoint.h"
#include "Map.h"
#include <sstream>

#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{


typedef vector<pair<double,double> > PointsVec;

class MapPublisher
{
public:
	MapPublisher(Map* pMap):mpMap(pMap) {
		n.param("dbscan_radius", radius, 0.1);
		n.param("dbscan_ep", ep, 3);
		n.param("points_topic", topic_out, string("points_map"));
		cone_map_pub = n.advertise<std_msgs::Float32MultiArray>(topic_out, 1000);

	};

	void PublishPoints()
	{
		PointsVec REDpoints, BLUEpoints;
		if(MakeConeMap(REDpoints, BLUEpoints))
		{
			// cone_map_msg msg;
			// msg.red = REDpoints;
			// msg.blue = BLUEpoints;
			// cone_map_pub.publish(msg);
			cout<< "got cones" << endl;
		}
	};

private:
	bool MakeConeMap(PointsVec &REDpoints, PointsVec &BLUEpoints)
	{
		// const int dim = 2; // TODO: maybe try with 3
		const vector<MapPoint*> &vpCs = mpMap->GetAllConePoints();
		int N = vpCs.size();

		// number of points from the map
		int NRED = 0, NBLUE = 0;
		// for all cone -> get matrices of RED and BLUE
		for(int i = 0; i < N; i++)
		{
			// postion in world
			cv::Mat pPos = vpCs[i]->GetWorldPos();
			float x = pPos.at<float>(0);
			float y = pPos.at<float>(1);
			// float z = pPos.at<float>(2); // no use for now
			std::pair<double,double> tmp(x,y);

			if (vpCs[i]->mnConeType == 0) // RED
			{
				REDpoints.push_back(tmp);
				cout << "red" << endl;
				NRED++;
			}
			else if(vpCs[i]->mnConeType == 1) // BLUE
			{
				BLUEpoints.push_back(tmp);
				NBLUE++;
			}
			else
				// TODO:change to ignore
				cout << "big problem, the map point given should be a cone\n";
		}

		if (!NRED && !NBLUE)
			return false;
		return true;
	}

	Map* mpMap;
	ros::NodeHandle n;
	ros::Publisher cone_map_pub;
	double radius;
	int ep;
	string topic_out;

};





} //namespace ORB_SLAM



#endif // MAPPUBLISHER_H