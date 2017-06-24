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
		params.push_back(new Parameter<bool>("filterCloud", &filterCloud, "filter cloud before creating transform, should improve results"));
	}
};

std::vector<std::string> LocalConfig::cameraTopics = boost::assign::list_of("/kinect1/sd/points")("/kinect2/depth_registered/points");
int LocalConfig::calibrationType = 0;
float LocalConfig::squareSize = 0.0245;
int LocalConfig::chessBoardWidth = 6;
int LocalConfig::chessBoardHeight = 9;
bool LocalConfig::saveTransform = true;
bool LocalConfig::filterCloud = false;


vector<Matrix4f> transforms;
std::vector<std::string> frameName;

boost::shared_ptr<tf::TransformBroadcaster> broadcaster;
boost::shared_ptr<tf::TransformListener> listener;

const float BAD_POINT = numeric_limits<float>::quiet_NaN();

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	int nCameras = LocalConfig::cameraTopics.size();
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
