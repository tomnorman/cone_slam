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

class MapPublisher
{
public:
	MapPublisher(Map* pMap);

	void PublishPoints(cv::Mat mOw);

private:
	void MakeConeMap();

	Map* mpMap;
	vector<vector<float>> YELLOWpoints, BLUEpoints;
	// ROS
	string topic_out;
	ros::NodeHandle n;
	ros::Publisher cone_map_pub;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H