#include <cmath>
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/median_filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_chessboard_pose.h"
#include "utils/my_exceptions.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"
#include "utils_cv.h"


////////////////////////
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <math.h>

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
	static std::vector<std::string> cameraTopics;
	static int calibrationType;
	static float squareSize;
	static int chessBoardWidth;
	static int chessBoardHeight;
	static bool filterCloud;
	static int chessboardPositions;

	LocalConfig() : Config() {
		params.push_back(new Parameter<std::vector<std::string> >("cameraTopics", &cameraTopics, "camera base topics. there should be at least two."));
		params.push_back(new Parameter<int>("calibrationType", &calibrationType, "0: Load from file calibration \n1:SVD calibration \n2: image calibration"));
		params.push_back(new Parameter<float>("squareSize", &squareSize, "the length (in meters) of the sides of the squares (if calibrationType != 0)"));
		params.push_back(new Parameter<int>("chessBoardWidth", &chessBoardWidth, "number of inner corners along the width of the chess board (if calibrationType != 0)"));
		params.push_back(new Parameter<int>("chessBoardHeight", &chessBoardHeight, "number of inner corners along the height of the chess board (if calibrationType != 0)"));
		params.push_back(new Parameter<bool>("filterCloud", &filterCloud, "filter cloud before creating transform, should maybe improve results"));
		params.push_back(new Parameter<int>("chessboardPositions", &chessboardPositions, "how many positions to calibrate the chessboard in"));
	}
};

std::vector<std::string> LocalConfig::cameraTopics = boost::assign::list_of("/kinect1/depth_registered/points");//("/kinect2/depth_registered/points");
int LocalConfig::calibrationType = 0;
float LocalConfig::squareSize = 0.0395;
int LocalConfig::chessBoardWidth = 4;
int LocalConfig::chessBoardHeight = 5;
bool LocalConfig::filterCloud = false;
int LocalConfig::chessboardPositions = 2;

vector<Matrix4f> transforms;
vector<ColorCloudPtr> corners;
vector<ColorCloudPtr> refCorners;
std::vector<std::string> frameName;
ros::Subscriber sub;
vector<bool> initialized(1, 2);
vector<float> positions;
int stage;
int nCameras;
int failureCount = 0;

boost::shared_ptr<tf::TransformBroadcaster> broadcaster;
boost::shared_ptr<tf::TransformListener> listener;

vector<sensor_msgs::PointCloud2> msg_out;
sensor_msgs::PointCloud2 msg_out1;

const float BAD_POINT = numeric_limits<float>::quiet_NaN();

ColorCloudPtr transformedCloud = ColorCloudPtr(new ColorCloud);

int getTopicIndex(string topic) {
	std::vector<string>::iterator it = std::find(LocalConfig::cameraTopics.begin(), LocalConfig::cameraTopics.end(), topic);
	if (it == LocalConfig::cameraTopics.end())
		return -1;
	else
		return std::distance(LocalConfig::cameraTopics.begin(), it);
}

ColorCloudPtr filterCloud(ColorCloudPtr in) {
	ColorCloudPtr out = ColorCloudPtr(new ColorCloud());

	ROS_INFO("Filtering cloud, might take a while");
	pcl::MedianFilter<ColorPoint> med;
	med.setInputCloud(in);
	med.setWindowSize(10);
	med.setMaxAllowedMovement(0.5);
	med.applyFilter(*out);

	return out;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input, const std::string &topic) {
	int index = getTopicIndex(topic);

	//make sure we haven't aleady added the chessboard points to this camera's calibration point cloud
	if(!initialized[index]) {
		ColorCloudPtr cloud_in(new ColorCloud());

		frameName[index] = input->header.frame_id;
		pcl::fromROSMsg(*input, *cloud_in);
		Matrix4f transform;
		int found_corners1 = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transform);
		if (found_corners1 == LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight) {
			if(failureCount > 0) cerr << "\rCamera " << index << " found chessboard!                                                                    ";
			cerr << endl;
			if(LocalConfig::filterCloud) cloud_in = filterCloud(cloud_in);

			ColorCloudPtr corner = chessBoardCorners(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);

			vector<int> badPoints;
			for (int i=0; i<corner->size(); i++) {
				if (!pointIsFinite(corner->at(i)))
					badPoints.push_back(i);
			}

			for (int i=(badPoints.size()-1); i>=0; i--) {
				corner->erase(corner->begin() + badPoints[i]);
			}

			*corners[index] += *corner;

			ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", index, found_corners1, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);

			failureCount = 0;
			initialized[index] = true;
		} else {
			if(failureCount > 25) {
				cerr << "\rCamera " << index << " can't find the chessboard.  Try repositioning it.";
			} else {
				cerr << "\rCamera " << index << " searching for chessboard...";
				++failureCount;
			}
		}

	}

}

//remove the 3 points that stand out most in each axis to give a less noisy scaling factor
ColorCloudPtr removeOutlierPoints(ColorCloudPtr in) {
	ColorCloudPtr out = ColorCloudPtr(new ColorCloud(*in));

	for(int j=0; j<3; ++j) {
		pcl::PointXYZRGB min, max;
		pcl::getMinMax3D (*out, min, max);
		for(int i=0; i<out->size(); ++i) {
			if(out->at(i).x == min.x || out->at(i).x == max.x
					|| out->at(i).x == min.y || out->at(i).x == max.y
					|| out->at(i).x == min.z || out->at(i).x == max.z) {
				out->erase(out->begin() + i);
			}

		}
	}

	return out;
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	nCameras = LocalConfig::cameraTopics.size();
	transforms.assign(nCameras, Matrix4f::Identity());

	ros::init(argc, argv,"kinectAlignment");
	ros::NodeHandle nh;

	ColorCloudPtr cloud_in(new ColorCloud());

	for (int i=0; i<nCameras; i++) {
		ColorCloudPtr placeHolder = ColorCloudPtr(new ColorCloud);
		initialized.push_back(false);
		frameName.push_back("");
		corners.push_back(placeHolder);
		refCorners.push_back(placeHolder);
	}

	initialized[0] = false;
	positions.resize(nCameras * 3 * 2);
	bool finished = false;

	//collect chessboard points
	for(stage = 0; stage < LocalConfig::chessboardPositions; ++stage) {
		for (int i=0; i<nCameras; i++) {
			sub = nh.subscribe<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], 1, boost::bind(&callback, _1, LocalConfig::cameraTopics[i]));
			while(!initialized[i]) {
				ros::spinOnce();
			}
		}

		for (int i=0; i<nCameras; i++) {
			initialized[i] = false;
		}
		finished = false;

		if(stage < LocalConfig::chessboardPositions-1) {
			cerr << endl;
			ROS_INFO ("Reposition the chessboard then hit ENTER to advance");

			while(true) {
				char c=getchar();
				cerr << c;
				if (c=='\n' || c==EOF) break;
			}
		}
	}

	//load transform from kinect calibration node
	for(int i=0; i<nCameras; ++i) {
		loadTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + frameName[i] + ".tf", transforms[i]);
	}

	//get max and min points to calculate scale
	pcl::transformPointCloud(*corners[0], *corners[0], transforms[0]);
	pcl::PointXYZRGB min0, max0;
	pcl::getMinMax3D (*removeOutlierPoints(corners[0]), min0, max0);



	for(int i=1; i<nCameras; ++i) {
		Matrix4f scale;

		pcl::transformPointCloud(*corners[i], *corners[i], transforms[i]);
		pcl::PointXYZRGB minI, maxI;
		pcl::getMinMax3D (*removeOutlierPoints(corners[i]), minI, maxI);

		scale << (max0.x-min0.x) / (maxI.x-minI.x), 0, 0, 0,
				0, (max0.y-min0.y) / (maxI.y-minI.y), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;


		cerr << "scale\n" << scale << endl;

		saveTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + frameName[i] + ".sc", scale);
		ROS_INFO("Scale info on camera %d is saved to file.", i);

	}

}
