#include "MapPublisher.h"

namespace ORB_SLAM2
{

MapPublisher::MapPublisher(Map* pMap) : mpMap(pMap) {
	n.param("/cone_slam/points_topic", topic_out, string("points_map"));
	cone_map_pub = n.advertise<std_msgs::Float32MultiArray>(topic_out, 1000);
}

void MapPublisher::PublishPoints(cv::Mat mOw)
{
	REDpoints.clear();
	BLUEpoints.clear();
	std_msgs::Float32MultiArray m;
	MakeConeMap();
	unsigned int NRED = REDpoints.size();
	unsigned int NBLUE = BLUEpoints.size();
	// cout << NRED << " RED cones, " << NBLUE << " blue cones.\n";

	/*
	create array msg:
		mOw,0
		NRED,NBLUE,0,0
		x1,y1,z1,color_enum
		...
		xi,yi,zi,color_enum
		...
	*/

	int mOwRows = mOw.rows; // should be 3
	int cols = 4;
	int rows = 2+NRED+NBLUE; // first row-mOw, second row-amount of cones, all else is points
	m.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m.layout.dim[0].label = "rows";
	m.layout.dim[0].size = rows;
	m.layout.dim[0].stride = cols*rows;
	m.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m.layout.dim[1].label = "cols";
	m.layout.dim[1].size = cols; // for i>0: data[i,3] = cone type
	m.layout.dim[1].stride = cols;

	// mOw [3x1]
	for (int i = 0; i < mOwRows; ++i)
		m.data.push_back(mOw.at<float>(0,i));
	m.data.push_back(static_cast<float>(0));

	// amount of points
	m.data.push_back(static_cast<float>(NRED));
	m.data.push_back(static_cast<float>(NBLUE));
	m.data.push_back(static_cast<float>(0));
	m.data.push_back(static_cast<float>(0));

	// REDpoints
	for (int i = 0; i < NRED; ++i)
		for (int j = 0; j < cols; ++j)
			m.data.push_back(REDpoints[i][j]); // x,y,z

	// BLUEpoints
	for (int i = 0; i < NBLUE; ++i)
		for (int j = 0; j < cols; ++j)
			m.data.push_back(BLUEpoints[i][j]); // x,y,z
	cone_map_pub.publish(m);
}

void MapPublisher::MakeConeMap()
{
	vector<vector<float>> points = mpMap->GetAllConePoints();
	// cout << "there are " << points.size() << " points\n";

	// for all cone -> get vectors of RED and BLUE
	for(auto &i : points)
	{
		// cout <<static_cast<int>(i.back()) << " point type\n";
		if (static_cast<int>(i.back()) == 0) // RED
			REDpoints.push_back(i);
		else if(static_cast<int>(i.back()) == 1) // BLUE
			BLUEpoints.push_back(i);
	}
}

} //namespace ORB_SLAM