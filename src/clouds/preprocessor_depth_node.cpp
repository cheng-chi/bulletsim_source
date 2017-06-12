#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "utils/config.h"
#include "clouds/utils_ros.h"
#include "clouds/cloud_ops.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using sensor_msgs::PointCloud2;
using namespace std;


struct LocalConfig: Config {
  static std::string outputTopic;
  static std::string nodeNS;
  static bool enableFarClip;
  static int farClipSampleSize;
  static bool displayCloud;
  static bool displayDepthMask;
  static int averageImages;
  static int depthThreshold;


  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("outputTopic", &outputTopic, "what topic the node should publish to"));
    params.push_back(new Parameter<string> ("nodeNS", &nodeNS, "node namespace"));
    params.push_back(new Parameter<bool> ("enableFarClip", &enableFarClip, "enable far plane clipping (kinect must be facing straight down to work)"));
    params.push_back(new Parameter<int> ("farClipSampleSize", &farClipSampleSize, "size of the are to sample for the far plane)"));
    params.push_back(new Parameter<bool> ("displayCloud", &displayCloud, "display the output point cloud"));
    params.push_back(new Parameter<bool> ("displayDepthMask", &displayDepthMask, "display the depth mask used to filter the point cloud"));
    params.push_back(new Parameter<int> ("averageImages", &averageImages, "number of depth maps to average for the background depth image"));
    params.push_back(new Parameter<int> ("depthThreshold", &depthThreshold, "how much closer than the background image something must be to be segmented"));
  }
};

string LocalConfig::outputTopic = "/kinect1/depth_registered/points";
string LocalConfig::nodeNS = "/preprocessor/kinect1";
bool LocalConfig::enableFarClip = false;
int LocalConfig::farClipSampleSize = 15;
bool LocalConfig::displayCloud = false;
bool LocalConfig::displayDepthMask = false;
int LocalConfig::averageImages = 20;
int LocalConfig::depthThreshold = 0;

const std::string cloudName = "rendered";

class DepthPreprocessorNode {
public:
	ros::Publisher m_pub;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	cv::Mat m_depthMask, m_backgroundDepth;
	bool m_cloudPending, m_depthPending;
	pcl::visualization::PCLVisualizer::Ptr m_visualizer;
	int m_imagesGathered;
	int m_maxDepth;
	vector<cv::Mat> m_backgroundImages;
	bool m_visualizer_init;

	ros::Subscriber subCloud;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;

	void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input) {
		pcl::fromROSMsg(*input, *m_cloud);

		if(m_depthMask.rows > 0) { //check if depthMask has been initialized
			//filter cloud based on depthMask
			for (int i=0; i<m_cloud->height; i++) {
				for (int j=0; j<m_cloud->width; j++) {
					if (m_depthMask.at<short>(i,j) == 0) {
						m_cloud->at(m_cloud->width*i+j).x =  numeric_limits<float>::quiet_NaN();
						m_cloud->at(m_cloud->width*i+j).y =  numeric_limits<float>::quiet_NaN();
						m_cloud->at(m_cloud->width*i+j).z =  numeric_limits<float>::quiet_NaN();
						m_cloud->at(m_cloud->width*i+j).r = 0;
						m_cloud->at(m_cloud->width*i+j).g = 0;
						m_cloud->at(m_cloud->width*i+j).b = 0;
					}
				}
			}
		}

		//display point cloud
		if(LocalConfig::displayCloud) {
			if(m_visualizer_init) {
				m_visualizer->updatePointCloud(m_cloud, cloudName);
			} else {
				m_visualizer->setBackgroundColor (0, 0, 0);
				m_visualizer->initCameraParameters ();
				m_visualizer->setCameraPosition(0, 0, 0, 0, -1, 0, 0.8575);
				m_visualizer->addPointCloud<pcl::PointXYZRGB> (m_cloud, cloudName);
				m_visualizer_init = true;
			}
			m_visualizer->spinOnce();
		}

		m_cloudPending = true;


		//convert cloud to publishable format then publish it
		pcl::PCLPointCloud2 output;
		pcl::toROSMsg(*m_cloud, output);
		m_pub.publish(output);
	}

	void depthCB(const sensor_msgs::ImageConstPtr& in) {
		//convert raw input to usable Mat
		cv::Mat depth;
		cv_bridge::CvImageConstPtr pCvImage;
		pCvImage = cv_bridge::toCvShare(in, in->encoding);
		pCvImage->image.copyTo(depth);

		//gather depth samples to average
		if(m_imagesGathered < LocalConfig::averageImages) {
			depth.copyTo(m_backgroundImages[m_imagesGathered]);
			++m_imagesGathered;
		}

		//calculate average background depth
		if(m_imagesGathered == LocalConfig::averageImages) {
			cv::Mat average(m_backgroundImages[0].size().height, m_backgroundImages[0].size().width, CV_16UC1);
			vector<double> pixels(m_backgroundImages[0].rows * m_backgroundImages[0].cols);

			//add together all images and store the result in the pixels vector
			#pragma omp parallel for
			for (int i = 0; i < m_backgroundImages.size(); ++i) {
				int pixel = 0;
				for (int r = 0; r < m_backgroundImages[i].rows; ++r) {
					const uint16_t *itD = m_backgroundImages[i].ptr<uint16_t>(r);
					uint16_t *itA = average.ptr<uint16_t>(r);
					for (size_t c = 0; c < (size_t) m_backgroundImages[i].cols; ++c, ++itD, ++pixel) {
						pixels[pixel] += *itD;
					}
				}
			}

			//divide the elements in the pixels vector by the number of images averaged
			int pixel = 0;
			#pragma omp parallel for
			for (int r = 0; r < average.rows; ++r) {
				uint16_t *itA = average.ptr<uint16_t>(r);
				for (size_t c = 0; c < (size_t) average.cols; ++c, ++itA, ++pixel) {
					*itA = pixels[pixel]/LocalConfig::averageImages;
				}
			}

			average.copyTo(m_backgroundDepth);

			//calculate the far plane cutoff
			if(LocalConfig::enableFarClip) {
				int denominator = 0;
				for (int r = m_backgroundDepth.rows/2 - LocalConfig::farClipSampleSize; r < m_backgroundDepth.rows/2 + LocalConfig::farClipSampleSize; ++r) {
					uint16_t *itA = m_backgroundDepth.ptr<uint16_t>(r);
					for (size_t c = 0; c < (size_t) m_backgroundDepth.cols; ++c, ++itA) {
						if(*itA != 0) {
							m_maxDepth += *itA;
							++denominator;
						}

					}
				}
				m_maxDepth /= denominator;
				m_maxDepth += LocalConfig::depthThreshold;
			} else {
				m_maxDepth = 65535;
			}

			++m_imagesGathered;
		}

		//create depth mask
		if(m_imagesGathered > LocalConfig::averageImages) {
			depth = filterDepth(depth);

			cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5, 5));
			cv::erode( depth, depth, element );
			cv::dilate( depth, m_depthMask, element );

			if(LocalConfig::displayDepthMask) {
				cv::imshow("Depth Mask", m_depthMask);
			}


		}

		m_depthPending = true;
		int key = cv::waitKey(1);
	}

	//filter depth based on background depth
	cv::Mat filterDepth(cv::Mat in) {
		cv::Mat out(in.rows, in.cols, CV_16UC1);
		#pragma omp parallel for
		for(int r = 0; r < in.rows; ++r) {
		  const uint16_t *inPtr = in.ptr<uint16_t>(r);
		  const uint16_t *backgroundPtr = m_backgroundDepth.ptr<uint16_t>(r);
		  uint16_t *outPtr = out.ptr<uint16_t>(r);
		  for(size_t c = 0; c < (size_t)in.cols; ++c, ++inPtr, ++backgroundPtr, ++outPtr) {
			  if(*inPtr + LocalConfig::depthThreshold < *backgroundPtr && *inPtr > 0 && *inPtr < m_maxDepth) {
				  *outPtr = 65535;
			  } else {
				  *outPtr = 0;
			  }
		  }
		}
		return out;
	}

	DepthPreprocessorNode(ros::NodeHandle& nh) :
		subCloud(nh.subscribe<sensor_msgs::PointCloud2>("/kinect1/sd/points", 5, &DepthPreprocessorNode::cloudCB, this)),
		it(nh),
		sub(it.subscribe("/kinect1/sd/image_depth", 1, &DepthPreprocessorNode::depthCB, this)),

		m_pub(nh.advertise<pcl::PCLPointCloud2>(LocalConfig::outputTopic, 10)),
		m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		m_cloudPending(false),
		m_depthPending(false),
		m_visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer")),
		m_backgroundImages(LocalConfig::averageImages),

		m_imagesGathered(0),
		m_maxDepth(65535),
		m_visualizer_init(false) {}


};

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(GeneralConfig());
	parser.read(argc, argv);

	ros::init(argc, argv, "point_cloud_processor");
	ros::NodeHandle nh(LocalConfig::nodeNS);

	DepthPreprocessorNode tp(nh);

	ros::spin();
}

