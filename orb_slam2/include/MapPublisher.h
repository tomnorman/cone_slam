#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <vector>
#include <iostream>

#include "MapPoint.h"
#include "Map.h"
#include <sstream>

#include <ros/ros.h>

#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{


typedef vector<vector<float> > PointsVec;

class MapPublisher
{
public:
	MapPublisher(Map* pMap):mpMap(pMap) {
		n.param("/cone_slam/points_topic", topic_out, string("points_map"));
		cone_map_pub = n.advertise<std_msgs::Float32MultiArray>(topic_out, 1000);
	};

	void PublishPoints(cv::Mat mOw)
	{
		REDpoints.clear();
		BLUEpoints.clear();
		std_msgs::Float32MultiArray m;
		MakeConeMap();
		unsigned int NRED = REDpoints.size();
		unsigned int NBLUE = BLUEpoints.size();

		/*
		create array msg:
			mOw,0
			x1,y1,z1,color_enum
			...
			xi,yi,zi,color_enum
			...
		*/

		int rows = mOw.rows; // should be 3
		int cols = mOw.cols; // should be 1
		m.layout.dim.push_back(std_msgs::MultiArrayDimension());
		m.layout.dim[0].label = "rows";
		m.layout.dim[0].size = 4*(rows+NRED+NBLUE); // first row1 is the mOw, all else is points
		m.layout.dim[0].stride = 4*(rows+NRED+NBLUE);
		m.layout.dim.push_back(std_msgs::MultiArrayDimension());
		m.layout.dim[1].label = "cols";
		m.layout.dim[1].size = 4;
		m.layout.dim[1].stride = 4;

		// mOw
		for (int i = 0; i < rows; ++i)
		{
			for (int j = 0; j < cols; ++j)
			{
				m.data.push_back(mOw.at<float>(i,j));
			}
		}
		m.data.push_back(float(0.0)); // data[0,3] = 0

		// REDpoints
		for (int i = 0; i < NRED; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				m.data.push_back(REDpoints[i][j]); // x,y,z
			}
			m.data.push_back(float(0.0));
		}

		// BLUEpoints
		for (int i = 0; i < NBLUE; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				m.data.push_back(BLUEpoints[i][j]); // x,y,z
			}
			m.data.push_back(float(1.0));
		}

		cone_map_pub.publish(m);
	};

private:
	void MakeConeMap()
	{
		// const int dim = 2; // TODO: maybe try with 3
		const vector<MapPoint*> &vpCs = mpMap->GetAllConePoints();
		int N = vpCs.size();

		// for all cone -> get matrices of RED and BLUE
		for(int i = 0; i < N; i++)
		{
			// postion in world
			cv::Mat pPos = vpCs[i]->GetWorldPos();
			float x = pPos.at<float>(0);
			float y = pPos.at<float>(1);
			float z = pPos.at<float>(2);
			vector<float> tmp = {x,y,z};

			if (vpCs[i]->mnConeType == 0) // RED
				REDpoints.push_back(tmp);
			else if(vpCs[i]->mnConeType == 1) // BLUE
				BLUEpoints.push_back(tmp);
			else
				// TODO:change to ignore
				cout << "big problem, the map point given should be a cone\n";
		}
	}

	Map* mpMap;
	PointsVec REDpoints, BLUEpoints;
	// ROS
	string topic_out;
	ros::NodeHandle n;
	ros::Publisher cone_map_pub;

};





} //namespace ORB_SLAM



#endif // MAPPUBLISHER_H