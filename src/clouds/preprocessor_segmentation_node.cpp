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
	static std::string inputTopic;
	static std::string outputTopic;
	static std::string nodeNS;
	static bool enableFarClip;
	static int farClipSampleSize;
	static bool displayCloud;
	static bool displayColorMask;
	static bool displayDepthMask;
	static bool displayLowMask;
	static bool displayHighMask;
	static bool displayThresholdMask;
	static bool displayCombinedMask;
	static int averageImages;
	static int depthThreshold;
	static int kSamples;
	static float deviations;
	static float r;
	static int n;
	static float clusterTolerance;
	static int clusterMinSize;
	static float downsampleAmount;
	static bool displayBackgroundDepth;


	LocalConfig() :
		Config() {
			params.push_back(new Parameter<string> ("inputTopic", &inputTopic, "what topic the node should read from"));
			params.push_back(new Parameter<string> ("outputTopic", &outputTopic, "what topic the node should publish to"));
			params.push_back(new Parameter<string> ("nodeNS", &nodeNS, "node namespace"));
			params.push_back(new Parameter<bool> ("enableFarClip", &enableFarClip, "enable far plane clipping (kinect must be facing straight down to work)"));
			params.push_back(new Parameter<int> ("farClipSampleSize", &farClipSampleSize, "size of the are to sample for the far plane)"));
			params.push_back(new Parameter<bool> ("displayCloud", &displayCloud, "display the output point cloud"));
			params.push_back(new Parameter<bool> ("displayColorMask", &displayColorMask, "display the color mask used to filter the point cloud"));
			params.push_back(new Parameter<bool> ("displayDepthMask", &displayDepthMask, "display the depth mask used to filter the point cloud"));
			params.push_back(new Parameter<bool> ("displayLowMask", &displayLowMask, "display the low mask used to filter the point cloud"));
			params.push_back(new Parameter<bool> ("displayHighMask", &displayHighMask, "display the high mask used to filter the point cloud"));
			params.push_back(new Parameter<bool> ("displayThresholdMask", &displayThresholdMask, "display the threshold mask used to filter the point cloud"));
			params.push_back(new Parameter<bool> ("displayCombinedMask", &displayCombinedMask, "display the combined mask used to filter the point cloud"));
			params.push_back(new Parameter<bool> ("displayBackgroundDepth", &displayBackgroundDepth, "display the background depth used to filter the point cloud"));
			params.push_back(new Parameter<int> ("averageImages", &averageImages, "number of depth maps to average for the background depth image"));
			params.push_back(new Parameter<int> ("depthThreshold", &depthThreshold, "how much closer than the background image something must be to be segmented"));
			params.push_back(new Parameter<int> ("kSamples", &kSamples, "K samples for pcl noise removal"));
			params.push_back(new Parameter<float> ("deviations", &deviations, "standard deviations for pcl noise removal"));
			params.push_back(new Parameter<float> ("r", &r, "r"));
			params.push_back(new Parameter<int> ("n", &n, "n"));
			params.push_back(new Parameter<float> ("clusterTolerance", &clusterTolerance, "clusterTolerance"));
			params.push_back(new Parameter<int> ("clusterMinSize", &clusterMinSize, "clusterMinSize"));
			params.push_back(new Parameter<float> ("downsampleAmount", &downsampleAmount, "downsample leaf size"));
		}
};

string LocalConfig::inputTopic = "/kinect1/sd/points";
string LocalConfig::nodeNS = "/preprocessor/kinect1";
string LocalConfig::outputTopic = "/preprocessor/kinect1/points";
bool LocalConfig::enableFarClip = false;
int LocalConfig::farClipSampleSize = 15;
bool LocalConfig::displayCloud = false;
bool LocalConfig::displayDepthMask = false;
bool LocalConfig::displayColorMask = false;
bool LocalConfig::displayLowMask = false;
bool LocalConfig::displayHighMask = false;
bool LocalConfig::displayThresholdMask = false;
bool LocalConfig::displayCombinedMask = false;
bool LocalConfig::displayBackgroundDepth = false;
int LocalConfig::averageImages = 20;
int LocalConfig::depthThreshold = 100;
int LocalConfig::kSamples = 10;
float LocalConfig::deviations = 1.5;
float LocalConfig::r = 0.025;
int LocalConfig::n = 10;
float LocalConfig::downsampleAmount = 0.006;
float LocalConfig::clusterTolerance = 0.03;
int LocalConfig::clusterMinSize = 100;


const std::string CLOUD_NAME = "rendered";
const int minx=175, maxx=255, miny=100, maxy=255, minz=0, maxz=255;
const float BAD_POINT = numeric_limits<float>::quiet_NaN();
const Mat DEPTH_KERNEL = getStructuringElement( MORPH_CROSS, Size(3, 3));
const Mat ERODE_KERNEL = getStructuringElement( MORPH_RECT, Size(32, 32));
const int MIN_L = 0, MAX_L = 255, MIN_A = 115, MAX_A = 255, MIN_B = 0, MAX_B = 255;

class PreprocessorSegmentationNode {
public:
	ros::Publisher m_cloudPub, m_imagePub, m_depthPub;
	ColorCloudPtr m_cloud, m_highCloud;
	Mat m_backgroundDepth, m_depth, m_color, m_depthMask, m_colorMask, m_highMask, m_lowMask, m_thresholdMask, m_combinedMask;
	pcl::visualization::PCLVisualizer::Ptr m_visualizer;
	int m_imagesGathered;
	int m_maxDepth;
	vector<Mat> m_backgroundImages;
	bool m_visualizer_init, m_backgroundDepth_init;

	ros::Subscriber m_sub;


	void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input) {
		pcl::fromROSMsg(*input, *m_cloud);
		pcl::copyPointCloud(*m_cloud, *m_highCloud);

		m_color = toCVMatImage(m_cloud);
		m_depth = toCVMatDepthImage(m_cloud);

		if(!m_backgroundDepth_init) {
			//create the background depth image
			createBackgroundDepth(m_depth);
		} else {
			m_colorMask = createColorMask(m_color); //looks for red color
			m_depthMask = createDepthMask(m_depth);
			m_thresholdMask = createThresholdMask(m_depthMask); //separates rope into parts above and on the table
			m_highMask = m_depthMask > 1200;
			m_highMask &= m_thresholdMask > 0; //rope lifted in the air
			m_lowMask = m_depthMask > 0;
			m_lowMask &= m_thresholdMask == 0; //rope lying on the table
			m_combinedMask = createCombinedMask(m_colorMask, m_lowMask, m_highMask);

			if(LocalConfig::displayColorMask) imshow("Color Mask", m_colorMask);
			if(LocalConfig::displayDepthMask) imshow("Depth Mask", m_depthMask);
			if(LocalConfig::displayHighMask) imshow("High Mask", m_highMask);
			if(LocalConfig::displayLowMask) imshow("High Mask", m_lowMask);
			if(LocalConfig::displayThresholdMask) imshow("Threshold Mask", m_thresholdMask);
			if(LocalConfig::displayCombinedMask) imshow("Combined Mask", m_combinedMask);
			if(LocalConfig::displayBackgroundDepth) imshow("Background Depth", m_backgroundDepth);


			for (int i=0; i<m_cloud->height; ++i) {
				for (int j=0; j<m_cloud->width; ++j) {
					if (m_combinedMask.at<uint8_t>(i,j) == 0) {
						m_cloud->at(m_cloud->width*i+j).x =  BAD_POINT;
						m_cloud->at(m_cloud->width*i+j).y =  BAD_POINT;
						m_cloud->at(m_cloud->width*i+j).z =  BAD_POINT;
						m_cloud->at(m_cloud->width*i+j).r = 0;
						m_cloud->at(m_cloud->width*i+j).g = 0;
						m_cloud->at(m_cloud->width*i+j).b = 0;
					}
				}
			}

			vector<int> indices;
			pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, indices);

			m_cloud = removeOutliers(m_cloud, LocalConfig::deviations, LocalConfig::kSamples);
			m_cloud = downsampleCloud(m_cloud, LocalConfig::downsampleAmount);
//			m_cloud = removeRadiusOutliers(m_cloud, LocalConfig::r, LocalConfig::n);
			m_cloud = clusterFilter(m_cloud, LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);


			if(LocalConfig::displayCloud) {
				displayCloud(m_cloud);
			}

			cvtColor(m_color, m_color, CV_BGR2BGRA);
			vector<Mat> channels;
			split(m_color, channels);
			channels[3] = m_combinedMask > 0;
			merge(channels, m_color);

			cv_bridge::CvImage image_msg;
			image_msg.header = input->header;
			image_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
			image_msg.image = m_color;


			cv_bridge::CvImage depth_msg;
			depth_msg.header = input->header;
			depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			depth_msg.image = m_depth;

			m_imagePub.publish(image_msg.toImageMsg());
			m_depthPub.publish(depth_msg.toImageMsg());

			sensor_msgs::PointCloud2 msg_out;
			pcl::toROSMsg(*m_cloud, msg_out);
			msg_out.header = input->header;
			m_cloudPub.publish(msg_out);

		}

		int key = waitKey(1);
	}

	Mat createColorMask(Mat color) {
		Mat red = colorSpaceMask(color, 0, 255, 160, 255, 0, 255, CV_BGR2Lab);
		Mat green = colorSpaceMask(color, 50, 100, 0, 255, 0, 255, CV_BGR2HSV);

		return red;// + green;
	}

	void createBackgroundDepth(Mat depth) {
		depth.convertTo(depth, CV_16UC1, 30000.0);

		if(m_imagesGathered < LocalConfig::averageImages) {
			if(m_imagesGathered == 0) {
				ROS_INFO("Gathering background depth samples");
			}

			//gather background images
			depth.copyTo(m_backgroundImages[m_imagesGathered]);
			++m_imagesGathered;
		} else {
			//calculate average background depth from gathered samples
			Mat average;
			m_backgroundImages[0].convertTo(average, CV_32SC1);
			for(int i = 1; i < m_backgroundImages.size(); ++i) {
				Mat temp;
				m_backgroundImages[i].convertTo(temp, CV_32SC1);
				average += temp;
			}
			average.convertTo(m_backgroundDepth, CV_16UC1, 1.0/float(LocalConfig::averageImages));

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
				m_maxDepth += 10*LocalConfig::depthThreshold;
			} else {
				m_maxDepth = 65535;
			}
			m_backgroundDepth_init = true;

			ROS_INFO("Created background depth image");
		}
	}

	Mat createDepthMask(Mat depth) {
		depth = filterDepth(depth, false);

		return depth;
	}

	Mat createHighMask(Mat depthMask) {
		Mat high = depthMask > LocalConfig::depthThreshold*14;
		return high;
	}

	Mat createThresholdMask(Mat highMask) {
		Mat out = highMask > 1800;
		dilate(out, out, getStructuringElement( MORPH_RECT, Size(17, 17)));
		return out;
	}

	Mat createCombinedMask(Mat color, Mat low, Mat high) {
		Mat combined = high + low;
		combined &= color;

		dilate(combined, combined, getStructuringElement( MORPH_ELLIPSE, Size(2, 2)));
		erode(combined, combined, getStructuringElement( MORPH_ELLIPSE, Size(2, 2)));

		return combined;
	}

	//filter depth based on background depth
	Mat filterDepth(Mat in, bool filterColor = true, int threshold = LocalConfig::depthThreshold) {
		in.convertTo(in, CV_16UC1, 30000);
		Mat out(in.rows, in.cols, CV_16UC1);
		out.copyTo(m_highMask);
		#pragma omp parallel for
		for(int r = 0; r < in.rows; ++r) {
		  const uint16_t *inPtr = in.ptr<uint16_t>(r);
		  const uint16_t *backgroundPtr = m_backgroundDepth.ptr<uint16_t>(r);
		  const uint8_t *colorPtr = m_colorMask.ptr<uint8_t>(r);
		  uint16_t *outPtr = out.ptr<uint16_t>(r);
		  for(size_t c = 0; c < (size_t)in.cols; ++c, ++inPtr, ++backgroundPtr, ++outPtr, ++colorPtr) {
			  if((*colorPtr > 0 || !filterColor) && *inPtr + threshold < *backgroundPtr && *inPtr > 0 && *inPtr < m_maxDepth) {
				  *outPtr = *backgroundPtr - *inPtr;
			  } else {
				  *outPtr = 0;
			  }
		  }
		}
		return out;
	}

	void displayCloud(ColorCloudPtr in) {
		if(m_visualizer_init) {
			m_visualizer->updatePointCloud(in, CLOUD_NAME);
		} else {
			m_visualizer->setBackgroundColor (0, 0, 0);
			m_visualizer->initCameraParameters ();
			m_visualizer->setCameraPosition(0, 0, 0, 0, -1, 0, 0.8575);
			m_visualizer->addPointCloud<ColorPoint> (in, CLOUD_NAME);
			m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, CLOUD_NAME);
			m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, CLOUD_NAME);

			m_visualizer_init = true;
		}
		m_visualizer->spinOnce();
	}

	PreprocessorSegmentationNode(ros::NodeHandle& nh) :
		m_sub(nh.subscribe<sensor_msgs::PointCloud2>(LocalConfig::inputTopic, 5, &PreprocessorSegmentationNode::cloudCB, this)),

		m_cloudPub(nh.advertise<pcl::PCLPointCloud2>(LocalConfig::outputTopic, 10)),
		m_imagePub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/image", 5)),
		m_depthPub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/depth", 5)),


		m_cloud(new ColorCloud),
		m_highCloud(new ColorCloud),
		m_visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer")),
		m_backgroundImages(LocalConfig::averageImages),

		m_imagesGathered(0),
		m_maxDepth(255),
		m_visualizer_init(false),
		m_backgroundDepth_init(false) {}


};

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(GeneralConfig());
	parser.read(argc, argv);

	ros::init(argc, argv, "point_cloud_processor");
	ros::NodeHandle nh(LocalConfig::nodeNS);

	PreprocessorSegmentationNode tp(nh);

	ros::spin();
}

