#include <boost/assign/list_of.hpp>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>

#include <iostream>
#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "utils/config.h"
#include "clouds/utils_ros.h"
#include "clouds/utils_cv.h"
#include "clouds/cloud_ops.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using sensor_msgs::PointCloud2;
using namespace std;
using namespace cv;

struct LocalConfig: Config {
	static std::vector<std::string> cameraTopics;
	static std::string outputTopic;
	static std::string nodeNS;
	static bool displayMasks;
	static int kSamples;
	static float deviations;
	static float clusterTolerance;
	static int clusterMinSize;
	static float downsampleAmount;
	static int maskSize;
	static float lowThreshold;
	static float highThreshold;
	static int method;
	static bool inpaintBackgroundMask;
	static bool useBackgroundMask;
	static bool isKinect2;
	static bool usingMultipleKinects;

	LocalConfig() :
		Config() {
			params.push_back(new Parameter<std::vector<std::string> > ("cameraTopics", &cameraTopics, "what camera topics the node should read from"));
			params.push_back(new Parameter<string> ("outputTopic", &outputTopic, "what topic the node should publish to"));
			params.push_back(new Parameter<string> ("nodeNS", &nodeNS, "node namespace"));
			params.push_back(new Parameter<bool> ("displayMasks", &displayMasks, "display the various masks used to filter the point cloud"));
			params.push_back(new Parameter<int> ("kSamples", &kSamples, "K samples for pcl noise removal"));
			params.push_back(new Parameter<float> ("deviations", &deviations, "standard deviations for pcl noise removal"));
			params.push_back(new Parameter<float> ("clusterTolerance", &clusterTolerance, "clusterTolerance"));
			params.push_back(new Parameter<int> ("clusterMinSize", &clusterMinSize, "clusterMinSize"));
			params.push_back(new Parameter<float> ("downsampleAmount", &downsampleAmount, "downsample leaf size"));
			params.push_back(new Parameter<int> ("maskSize", &maskSize, "mask size"));
			params.push_back(new Parameter<float> ("lowThreshold", &lowThreshold, "low threshold"));
			params.push_back(new Parameter<float> ("highThreshold", &highThreshold, "high threshold"));
			params.push_back(new Parameter<int> ("method", &method, "which segmentation method to use: 0=remove all points below threshold, 1=remove points below threshold only around lifted sections of rope"));
			params.push_back(new Parameter<bool> ("inpaintBackgroundMask", &inpaintBackgroundMask, "use inpainting to fill in missing portions of the background mask"));
			params.push_back(new Parameter<bool> ("useBackgroundMask", &useBackgroundMask, "use a background mask to improve heightmap accuracy"));
			params.push_back(new Parameter<bool> ("isKinect2", &isKinect2, "use kinect v2 parameters"));
			params.push_back(new Parameter<bool> ("usingMultipleKinects", &usingMultipleKinects, "changes parameters for multiple kinects"));
		}
};

std::vector<std::string> LocalConfig::cameraTopics = boost::assign::list_of("/kinect1/depth_registered/points");//("/kinect2/depth_registered/points");
string LocalConfig::nodeNS = "/preprocessor/kinect1";
string LocalConfig::outputTopic = "/preprocessor/kinect1/points";
bool LocalConfig::displayMasks = false;
int LocalConfig::kSamples = 10;
float LocalConfig::deviations = 1.5;
float LocalConfig::downsampleAmount = 0.007;
float LocalConfig::clusterTolerance = 0.05;
int LocalConfig::clusterMinSize = 30;
int LocalConfig::maskSize = 21;
float LocalConfig::lowThreshold = 0.008;
float LocalConfig::highThreshold = 0.03;
int LocalConfig::method = 0;
bool LocalConfig::inpaintBackgroundMask = true;
bool LocalConfig::useBackgroundMask = true;
bool LocalConfig::isKinect2 = false;
bool LocalConfig::usingMultipleKinects = false;



const std::string CLOUD_NAME = "rendered";
const float BAD_POINT = numeric_limits<float>::quiet_NaN();
int nCameras = LocalConfig::cameraTopics.size();

std::vector<Eigen::Matrix4f> transforms;
Eigen::Matrix4f inverseTransform;
std::vector< std::vector<float> > scales;

class PreprocessorSegmentationNode {
public:
	ros::Publisher m_cloudPub, m_imagePub, m_depthPub;
	std::vector<ColorCloudPtr> m_clouds;
	std::vector<bool> m_transforms_init, m_backgrounds_init;
	std::vector<Mat> m_backgrounds;
	std::vector<ros::Subscriber> m_subs;

	int getTopicIndex(string topic) {
		std::vector<string>::iterator it = std::find(LocalConfig::cameraTopics.begin(), LocalConfig::cameraTopics.end(), topic);
		if (it == LocalConfig::cameraTopics.end())
			return -1;
		else
			return std::distance(LocalConfig::cameraTopics.begin(), it);
	}


	void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input, const std::string &topic) {
		int index = getTopicIndex(topic);
		if(index == -1) {
			throw std::invalid_argument("camera topic not found");
		}

		pcl::fromROSMsg(*input, *m_clouds[index]);

		Mat color = toCVMatImage(m_clouds[index]);
		Mat depth = toCVMatDepthImage(m_clouds[index]);

		if(!m_transforms_init[index]) {
			//load camera transform matrix and scale info
			loadTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + string(input->header.frame_id) + ".tf", transforms[index]);
			if(index == 0)
				inverseTransform = Eigen::Matrix4f(transforms[index].inverse());
			loadScaleInfo(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + string(input->header.frame_id) + ".sc", scales[index]);
			m_transforms_init[index] =  true;
		}

		if(!m_backgrounds_init[index]) {
			if(LocalConfig::useBackgroundMask) {
				//create background z height image
				Mat mask = createBackgroundMask(color);

				for (int i=0; i<m_clouds[index]->height; ++i) {
					for (int j=0; j<m_clouds[index]->width; ++j) {
						if (mask.at<uint8_t>(i,j) == 0) {
							m_clouds[index]->at(m_clouds[index]->width*i+j).x =  BAD_POINT;
							m_clouds[index]->at(m_clouds[index]->width*i+j).y =  BAD_POINT;
							m_clouds[index]->at(m_clouds[index]->width*i+j).z =  BAD_POINT;
							m_clouds[index]->at(m_clouds[index]->width*i+j).r = 0;
							m_clouds[index]->at(m_clouds[index]->width*i+j).g = 0;
							m_clouds[index]->at(m_clouds[index]->width*i+j).b = 0;
						}
					}
				}

				pcl::transformPointCloud(*m_clouds[index], *m_clouds[index], transforms[index]);

				Mat backgroundHeightMap(depth.size(), CV_32FC1), inpaintMask(depth.size(), CV_8UC1);

				#pragma omp parallel for
				for (int r = 0; r < backgroundHeightMap.rows; ++r) {
					float *itD = backgroundHeightMap.ptr<float>(r);
					uint8_t *itB = inpaintMask.ptr<uint8_t>(r);
					for (size_t c = 0; c < (size_t) backgroundHeightMap.cols; ++c, ++itD, ++itB) {
						if(mask.at<uint8_t>(r, c) == 0 || abs(m_clouds[index]->at(c, r).z) > 0.05)
							*itB = 255;
						else {
							*itD = -m_clouds[index]->at(c, r).z;
							*itB = 0;
						}
					}
				}

				if(LocalConfig::inpaintBackgroundMask) {
					backgroundHeightMap.convertTo(backgroundHeightMap, CV_8UC1, 127.0/0.05, 127);
					inpaint(backgroundHeightMap, inpaintMask, backgroundHeightMap, 25, INPAINT_TELEA);
					backgroundHeightMap.convertTo(backgroundHeightMap, CV_32FC1, 0.05/127.0, -0.05);
				}

				backgroundHeightMap.copyTo(m_backgrounds[index]);
			} else {
				Mat backgroundHeightMap(depth.size(), CV_32FC1);
				backgroundHeightMap.setTo(cv::Scalar(0,0,0));
				backgroundHeightMap.copyTo(m_backgrounds[index]);
			}
			m_backgrounds_init[index] = true;
		}

		Mat colorMask = createColorMask(color);

		for (int i=0; i<m_clouds[index]->height; ++i) {
			for (int j=0; j<m_clouds[index]->width; ++j) {
				if (colorMask.at<uint8_t>(i,j) == 0) {
					m_clouds[index]->at(m_clouds[index]->width*i+j).x =  BAD_POINT;
					m_clouds[index]->at(m_clouds[index]->width*i+j).y =  BAD_POINT;
					m_clouds[index]->at(m_clouds[index]->width*i+j).z =  BAD_POINT;
					m_clouds[index]->at(m_clouds[index]->width*i+j).r = 0;
					m_clouds[index]->at(m_clouds[index]->width*i+j).g = 0;
					m_clouds[index]->at(m_clouds[index]->width*i+j).b = 0;
				}
			}
		}

		pcl::transformPointCloud(*m_clouds[index], *m_clouds[index], transforms[index]);

		Mat heightMap(depth.size(), CV_32FC1);

		#pragma omp parallel for
		for (int r = 0; r < heightMap.rows; ++r) {
			float *itD = heightMap.ptr<float>(r);
			const float *itB = m_backgrounds[index].ptr<float>(r);
			for (size_t c = 0; c < (size_t) heightMap.cols; ++c, ++itD, ++itB) {
				*itD = m_clouds[index]->at(c, r).z + *itB;
				m_clouds[index]->at(c, r).z += *itB;
			}
		}

		Mat low = heightMap < LocalConfig::lowThreshold;
		if(LocalConfig::method == 1) {
			Mat high = heightMap > LocalConfig::highThreshold;
			if(LocalConfig::isKinect2) {
				erode(high, high, getStructuringElement(MORPH_RECT, cv::Size(3, 3)));
			}
			dilate(high, high, getStructuringElement(MORPH_RECT, cv::Size(LocalConfig::maskSize, LocalConfig::maskSize)));
			low &= high;
		}

		colorMask &= (low == 0);
		erode(colorMask, colorMask, getStructuringElement(MORPH_RECT, cv::Size(3, 3)));

		for (int i=0; i<m_clouds[index]->height; ++i) {
			for (int j=0; j<m_clouds[index]->width; ++j) {
				if (colorMask.at<uint8_t>(i,j) == 0) {
					m_clouds[index]->at(m_clouds[index]->width*i+j).x =  BAD_POINT;
					m_clouds[index]->at(m_clouds[index]->width*i+j).y =  BAD_POINT;
					m_clouds[index]->at(m_clouds[index]->width*i+j).z =  BAD_POINT;
					m_clouds[index]->at(m_clouds[index]->width*i+j).r = 0;
					m_clouds[index]->at(m_clouds[index]->width*i+j).g = 0;
					m_clouds[index]->at(m_clouds[index]->width*i+j).b = 0;
				} else {
					m_clouds[index]->at(m_clouds[index]->width*i+j).x *= scales[index][0];
					m_clouds[index]->at(m_clouds[index]->width*i+j).y *= scales[index][1];
				}
			}
		}

		if(LocalConfig::displayMasks) {
			imshow("shadow removal", low);
			imshow("color mask", colorMask);
			imshow("height map", heightMap);
			imshow("background height map", m_backgrounds[index] * 20);
		}

		if(index == 0) {
			for(int i=1; i<nCameras; ++i) {
				*m_clouds[index] += *m_clouds[i];
			}

			vector<int> indices;
			pcl::removeNaNFromPointCloud(*m_clouds[index], *m_clouds[index], indices);

			if(LocalConfig::isKinect2) m_clouds[index] = removeOutliers(m_clouds[index], LocalConfig::deviations, LocalConfig::kSamples);
			m_clouds[index] = downsampleCloud(m_clouds[index], LocalConfig::downsampleAmount);
			if(!LocalConfig::usingMultipleKinects) m_clouds[index] = clusterFilter(m_clouds[index], LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

			pcl::transformPointCloud(*m_clouds[index], *m_clouds[index], inverseTransform);


			cvtColor(color, color, CV_BGR2BGRA);
			vector<Mat> channels;
			split(color, channels);
			channels[3] = colorMask > 0;
			merge(channels, color);

			cv_bridge::CvImage image_msg;
			image_msg.header = input->header;
			image_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
			image_msg.image = color;


			cv_bridge::CvImage depth_msg;
			depth_msg.header = input->header;
			depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			depth_msg.image = depth;

			m_imagePub.publish(image_msg.toImageMsg());
			m_depthPub.publish(depth_msg.toImageMsg());

			sensor_msgs::PointCloud2 msg_out;
			pcl::toROSMsg(*m_clouds[index], msg_out);
			msg_out.header = input->header;
			m_cloudPub.publish(msg_out);
		}

		int key = waitKey(1);
	}

	Mat createColorMask(Mat color) {
		Mat red = colorSpaceMask(color, 0, 255, 160, 255, 0, 255, CV_BGR2Lab);
		erode(red, red, getStructuringElement(MORPH_ELLIPSE, cv::Size(2, 2)));
		dilate(red, red, getStructuringElement(MORPH_ELLIPSE, cv::Size(2, 2)));
		return red;
	}

	Mat createBackgroundMask(Mat color) {
		Mat green = colorSpaceMask(color, 50, 100, 0, 255, 0, 255, CV_BGR2HSV);
		return green;
	}

	void init(ros::NodeHandle& nh) {
		for(int i=0; i<nCameras; ++i) {
			m_subs.push_back(nh.subscribe<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], 1, boost::bind(&PreprocessorSegmentationNode::cloudCB, this, _1, LocalConfig::cameraTopics[i])));
			ColorCloudPtr x(new ColorCloud);
			m_clouds.push_back(x);
			m_transforms_init.push_back(false);
			m_backgrounds_init.push_back(false);
			Mat empty;
			m_backgrounds.push_back(empty);
			transforms.reserve(nCameras);
			scales.resize(nCameras, std::vector<float>(2, 1));
			if(LocalConfig::isKinect2) {
				LocalConfig::maskSize = 100;
				LocalConfig::clusterTolerance = 0.1;
				LocalConfig::clusterMinSize = 200;
				LocalConfig::lowThreshold = 0.006;
				LocalConfig::method = 1;
			}
			if(LocalConfig::usingMultipleKinects) {
				LocalConfig::useBackgroundMask = false;
				LocalConfig::method = 1;
			}
		}
	}

	PreprocessorSegmentationNode(ros::NodeHandle& nh) :
		m_cloudPub(nh.advertise<pcl::PCLPointCloud2>(LocalConfig::nodeNS + "/points", 10)),
		m_imagePub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/image", 5)),
		m_depthPub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/depth", 5)) {}
};

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(GeneralConfig());
	parser.read(argc, argv);

	nCameras = LocalConfig::cameraTopics.size();

	ros::init(argc, argv, "preprocessor");
	ros::NodeHandle nh(LocalConfig::nodeNS);


	PreprocessorSegmentationNode tp(nh);
	tp.init(nh);

	ros::spin();
}
