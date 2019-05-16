#include "MapPublisher.h"

namespace ORB_SLAM2
{

MapPublisher::MapPublisher(Map* pMap) : mpMap(pMap) {
	n.param("/cone_slam/points_topic", topic_out, string("points_map"));
	cone_map_pub = n.advertise<std_msgs::Float32MultiArray>(topic_out, 1000);
}

void MapPublisher::PublishPoints(cv::Mat mOw)
{
	YELLOWpoints.clear();
	BLUEpoints.clear();
	std_msgs::Float32MultiArray m;
	MakeConeMap();
	unsigned int NYELLOW = YELLOWpoints.size();
	unsigned int NBLUE = BLUEpoints.size();
	// cout << NYELLOW << " YELLOW cones, " << NBLUE << " blue cones.\n";

	// Consts
	/*
	create array msg:
		mOw,0
		NYELLOW,NBLUE,0,0
		x1,y1,z1,color_enum
		...
		xi,yi,zi,color_enum
		...
	*/

	int mOwRows = mOw.rows; // should be 3
	int cols = 4;
	int rows = 2+NYELLOW+NBLUE; // first row-mOw, second row-amount of cones, all else is points
	m.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m.layout.dim[0].label = "rows";
	m.layout.dim[0].size = rows;
	m.layout.dim[0].stride = cols*rows;
	m.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m.layout.dim[1].label = "cols";
	m.layout.dim[1].size = cols; // for i>0: data[i,3] = cone type
	m.layout.dim[1].stride = cols;
	m.layout.data_offset = 0;

	// mOw [3x1]
	for (int i = 0; i < mOwRows; ++i)
		m.data.push_back(mOw.at<float>(0,i));
	m.data.push_back(static_cast<float>(0));

	// amount of points
	m.data.push_back(static_cast<float>(NYELLOW));
	m.data.push_back(static_cast<float>(NBLUE));
	m.data.push_back(static_cast<float>(0));
	m.data.push_back(static_cast<float>(0));

	// YELLOWpoints
	for (int i = 0; i < NYELLOW; ++i)
		for (int j = 0; j < cols; ++j)
			m.data.push_back(YELLOWpoints[i][j]); // x,y,z

	// BLUEpoints
	for (int i = 0; i < NBLUE; ++i)
		for (int j = 0; j < cols; ++j)
			m.data.push_back(BLUEpoints[i][j]); // x,y,z
	cone_map_pub.publish(m);
}

void MapPublisher::MakeConeMap()
{
	vector<vector<float>> points = mpMap->GetAllConePoints();
	//returns 4D vector
	// cout << "there are " << points.size() << " points\n";

	// for all cone -> get vectors of YELLOW and BLUE
	for(auto &i : points)
	{
		// cout <<static_cast<int>(i.back()) << " point type\n";
		if (static_cast<int>(i.back()) == 0) // YELLOW
			YELLOWpoints.push_back(i);
		else if(static_cast<int>(i.back()) == 1) // BLUE
			BLUEpoints.push_back(i);
	}
}

} //namespace ORB_SLAM