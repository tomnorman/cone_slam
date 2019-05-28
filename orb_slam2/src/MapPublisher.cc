#include "MapPublisher.h"

namespace ORB_SLAM2
{

MapPublisher::MapPublisher(Map* pMap) : mpMap(pMap) {
	n.param("/orb_slam2/points_topic", topic_out, string("points_map"));
	cones_map_pub = n.advertise<custom_msgs::slam_in>(topic_out, 1000);
}

void MapPublisher::PublishPoints(cv::Mat mOw, cv::Mat mRwc)
{
	if(mOw.empty() || mRwc.empty()) return;
	cout << mRwc << endl << mOw << endl;
	YELLOWpoints.clear();
	BLUEpoints.clear();
	custom_msgs::slam_in msg;
	MakeConeMap();
	const int NYELLOW = YELLOWpoints.size();
	const int NBLUE = BLUEpoints.size();
	msg.NYELLOW = NYELLOW;
	msg.NBLUE = NBLUE;
	msg.pos_x = mOw.at<float>(0,0);
	msg.pos_y = mOw.at<float>(0,1);
	msg.pos_z = mOw.at<float>(0,2);

	msg.normal_x = -mRwc.at<float>(0,2);
	msg.normal_y = -mRwc.at<float>(2,2);;
	msg.normal_z = -mRwc.at<float>(1,2);

	// YELLOWpoints
	for (int i = 0; i < NYELLOW; ++i)
	{
		msg.yellow_x.push_back(YELLOWpoints[i][0]);
		msg.yellow_y.push_back(YELLOWpoints[i][1]);
	}

	// BLUEpoints
	for (int i = 0; i < msg.NBLUE; ++i)
	{
		msg.blue_x.push_back(BLUEpoints[i][0]);
		msg.blue_y.push_back(BLUEpoints[i][1]);
	}
	cones_map_pub.publish(msg);
}

void MapPublisher::MakeConeMap()
{
	vector<vector<float>> points = mpMap->GetAllConePoints(); //returns 4D vector (x,y,z, type)

	//cout << "there are " << points.size() << " points\n";

	//for all cone -> get vectors of YELLOW and BLUE
	for(auto &i : points)
	{
		cout <<static_cast<int>(i.back()) << " point type\n";
		if (static_cast<int>(i.back()) == YELLOWC)
			YELLOWpoints.push_back(i);
		else if(static_cast<int>(i.back()) == BLUEC)
			BLUEpoints.push_back(i);
	}
}

} //namespace ORB_SLAM
