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
vector<ColorCloudPtr> corners;
vector<ColorCloudPtr> refCorners;
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

vector<sensor_msgs::PointCloud2> msg_out;
sensor_msgs::PointCloud2 msg_out1;

const float BAD_POINT = numeric_limits<float>::quiet_NaN();

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
	med.setWindowSize(20);
	med.setMaxAllowedMovement(0.5);
	med.applyFilter(*out);

	return out;
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
				if(LocalConfig::filterCloud) cloud_in = filterCloud(cloud_in);



				corners[index] = chessBoardCorners(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);
				refCorners[index] = ColorCloudPtr(new ColorCloud);
				for (int i=0; i<LocalConfig::chessBoardHeight; i++) {
					for (int j=(LocalConfig::chessBoardWidth-1); j>=0; j--) {
						ColorPoint pt;
						pt.x = LocalConfig::squareSize * (j - ((float) LocalConfig::chessBoardWidth - 1.0)/2.0);
						pt.y = LocalConfig::squareSize * (i - ((float) LocalConfig::chessBoardHeight - 1.0)/2.0);
						pt.z = 0;
						refCorners[index]->push_back(pt);
					}
				}

				//Filter out the bad points from both point clouds (cloud_corners and cloudref_corners)
				vector<int> badPoints;
				for (int i=0; i<corners[index]->size(); i++) {
					if (!pointIsFinite(corners[index]->at(i)))
						badPoints.push_back(i);
				}

				for (int i=(badPoints.size()-1); i>=0; i--) {
					corners[index]->erase(corners[index]->begin() + badPoints[i]);
					refCorners[index]->erase(refCorners[index]->begin() + badPoints[i]);
				}




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
				if(LocalConfig::filterCloud) cloud_in = filterCloud(cloud_in);



				ColorCloudPtr corner = chessBoardCorners(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);
				pcl::transformPointCloud(*corner, *corner, transforms[index]);
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid (*corner, centroid);

				cerr << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << ", " << centroid(3) << endl;
				float offX = sqrt(centroid(0) * centroid(0) + centroid(2) * centroid(2)) * ((centroid(0) > 0) - (centroid(0) < 0));
				float offY = sqrt(centroid(1) * centroid(1) + centroid(2) * centroid(2)) * ((centroid(1) > 0) - (centroid(1) < 0));

				ColorCloudPtr refCorner = ColorCloudPtr(new ColorCloud);
				ColorCloudPtr tempCorner = ColorCloudPtr(new ColorCloud);

				pcl::copyPointCloud(*corner, *tempCorner);


				for (int i=0; i<LocalConfig::chessBoardHeight; i++) {
					for (int j=(LocalConfig::chessBoardWidth-1); j>=0; j--) {
						ColorPoint pt;
						pt.x = LocalConfig::squareSize * (j - ((float) LocalConfig::chessBoardWidth - 1.0)/2.0);
						pt.y = LocalConfig::squareSize * (i - ((float) LocalConfig::chessBoardHeight - 1.0)/2.0);
						pt.z = 0;
						refCorner->push_back(pt);
					}
				}

				for(int i=0; i<tempCorner->size(); ++i) {
					tempCorner->at(i).z = 0;
				}



				Matrix4f translation, rotation;

				translation <<	1, 0, 0, -centroid(0),
								0, 1, 0, -centroid(1),
								0, 0, 1, 0,
								0, 0, 0, 1;

				pcl::transformPointCloud(*tempCorner, *tempCorner, translation);




				pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
				estimation_svd.estimateRigidTransformation(*tempCorner, *refCorner, rotation);




				pcl::transformPointCloud(*refCorner, *refCorner, Matrix4f(rotation.inverse()));
				pcl::transformPointCloud(*refCorner, *refCorner, Matrix4f(translation.inverse()));

//				pcl::transformPointCloud(*corner, *corner, Matrix4f(rotation.inverse()));
//				pcl::transformPointCloud(*corner, *corner, Matrix4f(translation.inverse()));

//				for (int i=0; i<LocalConfig::chessBoardHeight; i++) {
//					for (int j=(LocalConfig::chessBoardWidth-1); j>=0; j--) {
//						ColorPoint pt;
//						pt.x = offX + LocalConfig::squareSize * (j - ((float) LocalConfig::chessBoardWidth - 1.0)/2.0);
//						pt.y = offY + LocalConfig::squareSize * (i - ((float) LocalConfig::chessBoardHeight - 1.0)/2.0);
//						pt.z = 0;
//						refCorner->push_back(pt);
//					}
//				}

//				for (int i=0; i<corner->size(); i++) {
//					ColorPoint pt;
//					pt.x = sqrt(corner->at(i).x * corner->at(i).x + corner->at(i).z * corner->at(i).z);
//					pt.y = sqrt(corner->at(i).y * corner->at(i).y + corner->at(i).z * corner->at(i).z);
//					pt.z = 0;
//					refCorner->push_back(pt);
//				}

				//Filter out the bad points from both point clouds (cloud_corners and cloudref_corners)
				vector<int> badPoints;
				for (int i=0; i<corner->size(); i++) {
					if (!pointIsFinite(corner->at(i)))
						badPoints.push_back(i);
				}

				for (int i=(badPoints.size()-1); i>=0; i--) {
					corner->erase(corner->begin() + badPoints[i]);
					refCorner->erase(refCorner->begin() + badPoints[i]);
				}

				pcl::transformPointCloud(*corner, *corner, Matrix4f(transforms[index].inverse()));
				*corners[index] += *corner;
				*refCorners[index] += *refCorner;







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

ColorCloudPtr removeErrorPoints(ColorCloudPtr in) {
	ColorCloudPtr out = ColorCloudPtr(new ColorCloud(*in));

	for(int j=0; j<2; ++j) {
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

	ros::init(argc, argv,"kinectCalibrate");
	ros::NodeHandle nh;

	ColorCloudPtr cloud_in(new ColorCloud());





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
			pcl::fromROSMsg(*msg_in, *cloud_in);

			if(LocalConfig::filterCloud) {
				cloud_in = filterCloud(cloud_in);
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
	case 2: {
		for (int i=0; i<nCameras; i++) {
			ColorCloudPtr placeHolder = ColorCloudPtr(new ColorCloud);
			initialized.push_back(false);
			frameName.push_back("");
			corners.push_back(placeHolder);
			refCorners.push_back(placeHolder);
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



//		sensor_msgs::PointCloud2 m;

		pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
		estimation_svd.estimateRigidTransformation(*corners[0], *refCorners[0], transforms[0]);

//
		pcl::transformPointCloud(*corners[0], *corners[0], transforms[0]);

		pcl::PointXYZRGB min0, max0;
		pcl::getMinMax3D (*removeErrorPoints(corners[0]), min0, max0);

//
//		pcl::toROSMsg(*corners[0], m);
//		m.header.frame_id = frameName[0];
//
//		msg_out.push_back(m);



		for(int i=1; i<nCameras; ++i) {
			pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
			estimation_svd.estimateRigidTransformation(*corners[i], *corners[0], transforms[i]);

			Matrix4f temp;

			if (transforms[i](2,3) < 0)
				transforms[i] = Vector4f(-1,1,-1,1).asDiagonal() * transforms[i];

			pcl::transformPointCloud(*corners[i], *corners[i], transforms[i]);
			pcl::PointXYZRGB minI, maxI;
			pcl::getMinMax3D (*removeErrorPoints(corners[i]), minI, maxI);

			temp <<	(max0.x-min0.x) / (maxI.x-minI.x), 0, 0, 0,
					0, (max0.y-min0.y) / (maxI.y-minI.y), 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;


			pcl::transformPointCloud(*corners[i], *corners[i], temp);

			transforms[i] = transforms[i] * temp;

			Matrix4f s;
			s << 0.5, 0, 0, 0,
			     0, 0.5, 0, 0,
			     0, 0, 1, 0,
				 0, 0, 0, 1;

			cerr << "scale? " << temp << endl;

//			transforms[i] = transforms[i] * s;

			Eigen::Matrix3f R = transforms[i].topLeftCorner(3,3);
			cerr << "estimated scale " << std::sqrt( (R.transpose() * R).trace() / 3.0 ) << std::endl;

//			pcl::transformPointCloud(*refCorners[index], *refCorners[index], Matrix4f(transforms[index].inverse()));
//			pcl::toROSMsg(*refCorners[index], msg_out1);
//			msg_out1.header = input->header;

			if (LocalConfig::saveTransform){
				saveTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + frameName[i] + ".tf", transforms[i]);
				ROS_INFO("Calibration on camera %d is saved to file.", i);
			}
		}

//		pcl::transformPointCloud(*corners[1], *corners[1], transforms[1]);
//		*corners[0] = *corners[0] + *corners[1];
		sensor_msgs::PointCloud2 m, m1, m2;

		pcl::toROSMsg(*corners[0], m);
		m.header.frame_id = "/ground";

		msg_out.push_back(m);

		pcl::toROSMsg(*corners[1], m1);
		m1.header.frame_id = "/ground";
		msg_out.push_back(m1);

		pcl::toROSMsg(*refCorners[0], m2);
		m2.header.frame_id = "/ground";

		msg_out.push_back(m2);





		//calculate scale info
//		for(int i=0; i<nCameras; ++i) {
//			float scaleX, scaleY, scaleZ;
//			if(i == 0) {
//				scaleX = 1;
//				scaleY = 1;
//				scaleZ = 1;
//			} else {
//				scaleX = positions[0] / positions[i * 3];
//				scaleY = positions[1] / positions[i * 3 + 1];
//				scaleZ = positions[2] / positions[i * 3 + 2];
//			}
//			cerr << "scale X for Camera " << i << ": " << scaleX << endl;
//			cerr << "scale Y for Camera " << i << ": " << scaleY << endl;
//			cerr << "scale Z for Camera " << i << ": " << scaleZ << endl;
//			saveScaleInfo(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + frameName[i] + ".sc", scaleX, scaleY, scaleZ);
//		}

		break;
	}
	// Invalid calibration type
	default:
		ROS_WARN("Calibration type %d is invalid.", LocalConfig::calibrationType);
	}

	broadcaster.reset(new tf::TransformBroadcaster());
	listener.reset(new tf::TransformListener());

	ros::Rate rate(30);
	ROS_INFO("Publish /ground transforms to topic /tf");

	ros::Publisher pub = nh.advertise<pcl::PCLPointCloud2>("corners1/points", 10);
	ros::Publisher pub1 = nh.advertise<pcl::PCLPointCloud2>("corners2/points", 10);
	ros::Publisher pub2 = nh.advertise<pcl::PCLPointCloud2>("corners3/points", 10);


	while (nh.ok()){
		try {
			for (int i=0; i<nCameras; i++) {
				broadcastKinectTransform(toBulletTransform((Affine3f) transforms[i]), frameName[i], "/ground", *broadcaster, *listener);
			}
			pub.publish(msg_out[0]);
			pub1.publish(msg_out[1]);
			pub2.publish(msg_out[2]);
			rate.sleep();
		} catch (tf::TransformException ex) {
			ROS_WARN("transforms not loaded yet!  Sleeping for 1 second");
			ros::Duration(1.0).sleep();
		}
	}

}
