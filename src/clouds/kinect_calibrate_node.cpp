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
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/median_filter.h>
#include <pcl/visualization/cloud_viewer.h>
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

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
	static std::vector<std::string> cameraTopics;
	static int calibrationType;
	static float squareSize;
	static int chessBoardWidth;
	static int chessBoardHeight;
	static bool saveTransform;
	static bool filterCloud;

	LocalConfig() : Config() {
		params.push_back(new Parameter<std::vector<std::string> >("cameraTopics", &cameraTopics, "camera base topics. there should be at least two."));
		params.push_back(new Parameter<int>("calibrationType", &calibrationType, "0: Load from file calibration \n1:SVD calibration \n2: image calibration"));
		params.push_back(new Parameter<float>("squareSize", &squareSize, "the length (in meters) of the sides of the squares (if calibrationType != 0)"));
		params.push_back(new Parameter<int>("chessBoardWidth", &chessBoardWidth, "number of inner corners along the width of the chess board (if calibrationType != 0)"));
		params.push_back(new Parameter<int>("chessBoardHeight", &chessBoardHeight, "number of inner corners along the height of the chess board (if calibrationType != 0)"));
		params.push_back(new Parameter<bool>("saveTransform", &saveTransform, "save chessboard <-> camera transforms to file"));
		params.push_back(new Parameter<bool>("filterCloud", &filterCloud, "filter cloud before creating transform, should maybe improve results"));
	}
};

std::vector<std::string> LocalConfig::cameraTopics = boost::assign::list_of("/kinect1/depth_registered/points")("/kinect2/sd/points");
int LocalConfig::calibrationType = 0;
float LocalConfig::squareSize = 0.0245;
int LocalConfig::chessBoardWidth = 6;
int LocalConfig::chessBoardHeight = 9;
bool LocalConfig::saveTransform = true;
bool LocalConfig::filterCloud = false;


vector<Matrix4f> transforms;
std::vector<std::string> frameName;
vector<ros::Subscriber> subscriptions;
vector<bool> initialized(1, 2);
vector<float> positions;
int stage;
int nCameras;

boost::shared_ptr<tf::TransformBroadcaster> broadcaster;
boost::shared_ptr<tf::TransformListener> listener;

const float BAD_POINT = numeric_limits<float>::quiet_NaN();

int getTopicIndex(string topic) {
	std::vector<string>::iterator it = std::find(LocalConfig::cameraTopics.begin(), LocalConfig::cameraTopics.end(), topic);
	if (it == LocalConfig::cameraTopics.end())
		return -1;
	else
		return std::distance(LocalConfig::cameraTopics.begin(), it);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input, const std::string &topic) {
	int index = getTopicIndex(topic);

	if(!initialized[index]) {
		ColorCloudPtr cloud_in(new ColorCloud());
		switch(stage) {

		case 0: {

			pcl::fromROSMsg(*input, *cloud_in);

			int found_corners = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transforms[index]);
			if (found_corners) {
				ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", index, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
				cerr << endl << "FRAME NAME " << index << ": " << input->header.frame_id;
				frameName[index] = input->header.frame_id;
				if (LocalConfig::saveTransform){
					saveTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + input->header.frame_id + ".tf", transforms[index]);
					ROS_INFO("Calibration on camera %d is saved to file.", index);
				}
				initialized[index] = true;
			} else {
				ROS_WARN("SVD calibration on camera %d failed. Camera %d sees %d/%d corners.", index, index, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
			}
			break;
		}
		case 1: {
			pcl::fromROSMsg(*input, *cloud_in);
			Matrix4f transform;
			int found_corners1 = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transform);
			if (found_corners1) {
				ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", index, found_corners1, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
				ColorCloudPtr temp(new ColorCloud());
				ColorPoint pt;
				pt.x = 0;
				pt.y = 0;
				pt.z = 0;
				temp->push_back(pt);
				pcl::transformPointCloud(*temp, *temp, Eigen::Matrix4f(transforms[index].inverse()));
				pcl::transformPointCloud(*temp, *temp, transform);

				positions[index * 3] = temp->at(0).x;
				positions[index * 3 + 1] = temp->at(0).y;
				positions[index * 3 + 2] = temp->at(0).z;
				for (int i=0; i<positions.size(); i++) {
					cerr << positions[i] << ", ";
				}
				initialized[index] = true;
			} else {
				ROS_WARN("SVD calibration on camera %d failed. Camera %d sees %d/%d corners.", index, index, found_corners1, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
			}

			break;
		}
		case 2: {
			float scaleX, scaleY;
			if(index == 0) {
				scaleX = 1;
				scaleY = 1;
			} else {
				scaleX = positions[0] / positions[index * 3];
				scaleY = positions[1] / positions[index * 3 + 1];
			}
			cerr << endl << "scale X: " << scaleX;
			cerr << endl << "scale Y: " << scaleY;
			saveScaleInfo(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + input->header.frame_id + ".sc", scaleX, scaleY);
			initialized[index] = true;
			break;
		}
		case 3: {
			pcl::fromROSMsg(*input, *cloud_in);
			Matrix4f transform;
			int found_corners1 = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transform);
			if (found_corners1) {
				ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", index, found_corners1, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
				ColorCloudPtr temp(new ColorCloud());
				ColorPoint pt;
				pt.x = 0;
				pt.y = 0;
				pt.z = 0;
				temp->push_back(pt);
				pcl::transformPointCloud(*temp, *temp, Eigen::Matrix4f(transforms[index].inverse()));
				pcl::transformPointCloud(*temp, *temp, transform);

				positions[index * 3 + nCameras * 3] = temp->at(0).x;
				positions[index * 3 + 1 + nCameras * 3] = temp->at(0).y;
				positions[index * 3 + 2 + nCameras * 3] = temp->at(0).z;
				for (int i=0; i<positions.size(); i++) {
					cerr << positions[i] << ", ";
				}
				initialized[index] = true;
			} else {
				ROS_WARN("SVD calibration on camera %d failed. Camera %d sees %d/%d corners.", index, index, found_corners1, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
			}
		}
		case 4: {
//			for(int i=1; i<nCameras; ++i) {
//				float posZ = positions[i * 3 + nCameras * 3 + 2];
//				float offX = positions[nCameras * 3] - positions[i * 3 + nCameras * 3];
//				float offY = positions[nCameras * 3 + 1] - positions[i * 3 + nCameras * 3 + 1];
//				cerr << endl << "pos Z: " << posZ;
//				cerr << endl << "off X: " << offX;
//				cerr << endl << "off Y: " << offY;
//				initialized[i] = true;
//			}
			initialized[index] = true;
			break;
		}
		default:
			ROS_WARN("Invalid stage %d.", stage);
		}

	}

}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	nCameras = LocalConfig::cameraTopics.size();
	transforms.assign(nCameras, Matrix4f::Identity());

	ros::init(argc, argv,"kinectCalibrate");
	ros::NodeHandle nh;

	switch (LocalConfig::calibrationType) {
	// Load from file calibration
	case 0:
		for (int i=0; i<nCameras; i++) {

			sensor_msgs::PointCloud2ConstPtr msg_in = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], nh);
			frameName.push_back(msg_in->header.frame_id);
			if (loadTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + msg_in->header.frame_id + ".tf", transforms[i])) {
				ROS_INFO("Load from file calibration on camera %d succeeded.", i);
			} else {
				ROS_WARN("Load from file calibration on camera %d failed. Make sure the files exist.", i);
			}
		}
		break;
	// SVD calibration
	case 1:
		for (int i=0; i<nCameras; i++) {
			initialized.push_back(false);
			frameName.push_back("");
			subscriptions.push_back(nh.subscribe<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], 1, boost::bind(&callback, _1, LocalConfig::cameraTopics[i])));
		}
		initialized[0] = false;
		positions.resize(nCameras * 3 * 2);
		bool finished;
		finished = false;
		for(stage = 0; stage < 3; ++stage) { //NUM STAGES
			while(!finished) {
				finished = initialized[0];
				for (int i=1; i<nCameras; i++) {
					finished &= initialized[i];
				}
				ros::spinOnce();
			}

			for (int i=0; i<nCameras; i++) {
				initialized[i] = false;
			}
			finished = false;

			int c;
			  puts ("press . to advance:");
			  do {
				c=getchar();
				putchar (c);
			  } while (c != '.');

			ROS_INFO("Advancing stage");
		}

		for(int i=0; i<frameName.size(); ++i) {
			cerr << "Frame Name " << i << ": " << frameName[i] << endl;
		}

		break;
	// Invalid calibration type
	default:
		ROS_WARN("Calibration type %d is invalid.", LocalConfig::calibrationType);
	}

	broadcaster.reset(new tf::TransformBroadcaster());
	listener.reset(new tf::TransformListener());

	ros::Rate rate(30);
	ROS_INFO("Publish /ground transforms to topic /tf");
	while (nh.ok()){
		try {
			for (int i=0; i<nCameras; i++) {
				broadcastKinectTransform(toBulletTransform((Affine3f) transforms[i]), frameName[i], "/ground", *broadcaster, *listener);
			}
			rate.sleep();
		} catch (tf::TransformException ex) {
			ROS_WARN("transforms not loaded yet!  Sleeping for 1 second");
			ros::Duration(1.0).sleep();
		}
	}

}
