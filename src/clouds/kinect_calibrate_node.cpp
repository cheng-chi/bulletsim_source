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

std::vector<std::string> LocalConfig::cameraTopics = boost::assign::list_of("/kinect1/depth_registered/points");//("/kinect2/depth_registered/points");
int LocalConfig::calibrationType = 0;
float LocalConfig::squareSize = 0.0245;
int LocalConfig::chessBoardWidth = 6;
int LocalConfig::chessBoardHeight = 9;
bool LocalConfig::saveTransform = true;
bool LocalConfig::filterCloud = false;


vector<Matrix4f> transforms;
std::vector<std::string> frameName;
//vector<ros::Subscriber> subscriptions;
ros::Subscriber sub;
vector<bool> initialized(1, 2);
vector<float> positions;
int stage;
int nCameras;
int failureCount = 0;

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

		//svd calibrate and save transform
		case 0: {

			pcl::fromROSMsg(*input, *cloud_in);

			int found_corners = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transforms[index]);
			if (found_corners) {
				if(failureCount > 0) cerr << "\rCamera " << index << " found chessboard!                                                                    ";
				cerr << endl;
				ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", index, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
				frameName[index] = input->header.frame_id;
				if (LocalConfig::saveTransform){
					saveTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + input->header.frame_id + ".tf", transforms[index]);
					ROS_INFO("Calibration on camera %d is saved to file.", index);
				}
				failureCount = 0;
				initialized[index] = true;
			} else {
				if(failureCount > 20) {
					cerr << "\rCamera " << index << " can't find the chessboard.  Try repositioning it.";
				} else {
					cerr << "\rCamera " << index << " searching for chessboard...";
					++failureCount;
				}
			}
			break;
		}
		//find how far the chessboard has moved in order to calculate scale information
		case 1: {
			pcl::fromROSMsg(*input, *cloud_in);
			Matrix4f transform;
			int found_corners1 = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transform);
			if (found_corners1) {
				if(failureCount > 0) cerr << "\rCamera " << index << " found chessboard!                                                                    ";
				cerr << endl;
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

			sensor_msgs::PointCloud2ConstPtr msg_in = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], nh);
			frameName.push_back(msg_in->header.frame_id);
			ColorCloudPtr cloud_in(new ColorCloud());
			pcl::fromROSMsg(*msg_in, *cloud_in);

			if(LocalConfig::filterCloud) {
				ROS_INFO("Filtering cloud, might take a while");
				cv::Mat color = toCVMatImage(cloud_in);
				cv::Mat mask = colorSpaceMask(color, 30, 120, 30, 120, 30, 120, CV_BGR2RGB);
				mask |= colorSpaceMask(color, 200, 255, 200, 255, 200, 255, CV_BGR2RGB);

				cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
				cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)));
				cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(60, 60)));

				for (int k=0; k<cloud_in->height; ++k) {
					for (int j=0; j<cloud_in->width; ++j) {
						if (mask.at<uint8_t>(k,j) == 0) {
							cloud_in->at(cloud_in->width*k+j).x = BAD_POINT;
							cloud_in->at(cloud_in->width*k+j).y = BAD_POINT;
							cloud_in->at(cloud_in->width*k+j).z = BAD_POINT;
							cloud_in->at(cloud_in->width*k+j).r = 0;
							cloud_in->at(cloud_in->width*k+j).g = 0;
							cloud_in->at(cloud_in->width*k+j).b = 0;
						}
					}
				}

				pcl::MedianFilter<ColorPoint> med;
				med.setInputCloud(cloud_in);
				med.setWindowSize(50);
				med.setMaxAllowedMovement(0.5);
				med.applyFilter(*cloud_in);
			}

			int found_corners = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transforms[i]);
			if (found_corners) {
				ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", i, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
			} else {
				ROS_WARN("SVD calibration on camera %d failed. Camera %d sees %d/%d corners.", i, i, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
			}

			if (LocalConfig::saveTransform){
				saveTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + msg_in->header.frame_id + ".tf", transforms[i]);
				ROS_INFO("Calibration on camera %d is saved to file.", i);
			}
		}
		break;
	//multiple kinect calibration and scale
	case 2:
		for (int i=0; i<nCameras; i++) {
			initialized.push_back(false);
			frameName.push_back("");
		}
		initialized[0] = false;
		positions.resize(nCameras * 3 * 2);
		bool finished;
		finished = false;
		for(stage = 0; stage < 2; ++stage) {
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

			if(stage < 1) {
				cerr << endl;
				ROS_INFO ("Reposition the chessboard then hit ENTER to advance");

				while(true) {
					char c=getchar();
					cerr << c;
					if (c=='\n' || c==EOF) break;
				}
			}
		}

		//calculate scale info
		for(int i=0; i<nCameras; ++i) {
			float scaleX, scaleY, scaleZ;
			if(i == 0) {
				scaleX = 1;
				scaleY = 1;
				scaleZ = 1;
			} else {
				scaleX = positions[0] / positions[i * 3];
				scaleY = positions[1] / positions[i * 3 + 1];
				scaleZ = positions[2] / positions[i * 3 + 2];
			}
			cerr << "scale X for Camera " << i << ": " << scaleX << endl;
			cerr << "scale Y for Camera " << i << ": " << scaleY << endl;
			cerr << "scale Z for Camera " << i << ": " << scaleZ << endl;
			saveScaleInfo(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + frameName[i] + ".sc", scaleX, scaleY, scaleZ);
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
