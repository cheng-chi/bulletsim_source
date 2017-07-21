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
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/convergence_criteria.h>

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
	static bool isKinect2;
	static float zOffset;
	static bool translateCloud;
	static bool removeOutliers;
	static int updateFrequency;

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
			params.push_back(new Parameter<bool> ("isKinect2", &isKinect2, "use kinect v2 parameters"));
			params.push_back(new Parameter<float> ("zOffset", &zOffset, "how high off the table the rope is (should be negative), needs to be set in order for shadow removal to work correctly"));
			params.push_back(new Parameter<bool> ("translateCloud", &translateCloud, "Whether of not to apply the z offset to the point cloud"));
			params.push_back(new Parameter<bool> ("removeOutliers", &removeOutliers, "run statistical outlier removal filter (slow)"));
			params.push_back(new Parameter<int> ("updateFrequency", &updateFrequency, "how often to update the registration transformations between kinects"));
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
int LocalConfig::method = 1;
bool LocalConfig::isKinect2 = false;
float LocalConfig::zOffset = -0.12;
bool LocalConfig::translateCloud = false;
bool LocalConfig::removeOutliers = false;
int LocalConfig::updateFrequency = 30;



const std::string CLOUD_NAME = "rendered";
const float BAD_POINT = numeric_limits<float>::quiet_NaN();
int nCameras = LocalConfig::cameraTopics.size();
int cycle = 0, totalPoints = 0;

std::vector<Eigen::Matrix4f> transforms, scales, alignments;
Eigen::Matrix4f inverseTransform;
vector<bool> transforms_init;

int getTopicIndex(string topic) {
	std::vector<string>::iterator it = std::find(LocalConfig::cameraTopics.begin(), LocalConfig::cameraTopics.end(), topic);
	if (it == LocalConfig::cameraTopics.end())
		return -1;
	else
		return std::distance(LocalConfig::cameraTopics.begin(), it);
}

class PreprocessorSegmentationNode {
public:
	ros::Publisher m_cloudPub, m_imagePub, m_depthPub;
	std::vector<ColorCloudPtr> m_clouds;
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

		//load transform and scale matrices
		if(!transforms_init[index]) {
			loadTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + input->header.frame_id + ".tf", transforms[index]);
			alignments[index] = Eigen::Matrix4f::Identity();

			if(index == 0) {
				inverseTransform = Eigen::Matrix4f(transforms[index].inverse());
				scales[index] = Eigen::Matrix4f::Identity();
			} else {
				loadTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms/" + input->header.frame_id + ".sc", scales[index]);
			}

			transforms_init[index] = true;
		}

		//transform all clouds to ground frame
		pcl::transformPointCloud(*m_clouds[index], *m_clouds[index], transforms[index]);
		pcl::transformPointCloud(*m_clouds[index], *m_clouds[index], scales[index]);

		//create height map
		Mat heightMap(depth.size(), CV_32FC1);

		#pragma omp parallel for
		for (int r = 0; r < heightMap.rows; ++r) {
			float *itD = heightMap.ptr<float>(r);
			for (size_t c = 0; c < (size_t) heightMap.cols; ++c, ++itD) {
				*itD = m_clouds[index]->at(c, r).z + LocalConfig::zOffset;
			}
		}

		//adjacent point removal
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
		if(!LocalConfig::removeOutliers) erode(colorMask, colorMask, getStructuringElement(MORPH_RECT, cv::Size(3, 3)));

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

		//display masks for debug purposes
		if(LocalConfig::displayMasks) {
			imshow("shadow removal", low);
			imshow("color mask", colorMask);
			imshow("height map", heightMap);
		}

		//register and combine kinect point clouds
		if(index == 0) {
			vector<int> indices1;
			pcl::removeNaNFromPointCloud(*m_clouds[index], *m_clouds[index], indices1);

			if(totalPoints == 0) {
				totalPoints = m_clouds[index]->size();
			}

			cerr << "\rtotalPoints:" << totalPoints << " currentPoints: " << m_clouds[index]->size() << " cycle: " << cycle << "      ";

			//only run icp once every updateFrequency frames and if there are enough points in the first kinect's cloud
			if(cycle == LocalConfig::updateFrequency && m_clouds[index]->size() > 0.6 * totalPoints) {
				++cycle;
				m_clouds[index] = downsampleCloud(m_clouds[index], LocalConfig::downsampleAmount);
				if(LocalConfig::removeOutliers) m_clouds[index] = removeOutliers(m_clouds[index], LocalConfig::deviations, LocalConfig::kSamples);

				for(int i=1; i<nCameras; ++i) {
					m_clouds[i] = downsampleCloud(m_clouds[i], LocalConfig::downsampleAmount);
					if(LocalConfig::removeOutliers) m_clouds[i] = removeOutliers(m_clouds[i], LocalConfig::deviations, LocalConfig::kSamples);

					vector<int> indices;
					pcl::removeNaNFromPointCloud(*m_clouds[i], *m_clouds[i], indices);

					pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
					Eigen::Matrix4f T;
					icp.setInputSource (m_clouds[i]);
					icp.setInputTarget (m_clouds[index]);
					icp.setMaxCorrespondenceDistance(0.05);
					ColorCloud c;
					icp.align (c);
//					cerr << "\rhas converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
					alignments[i] = icp.getFinalTransformation();

				}

				cycle = 0;

			} else if(cycle < LocalConfig::updateFrequency) {
				++cycle;
			}

			//transform and combine clouds
			for(int i=1; i<nCameras; ++i) {
				ColorCloud t;
				pcl::transformPointCloud(*m_clouds[i], t, alignments[i]);
				*m_clouds[index] += t;
			}

			vector<int> indices;
			pcl::removeNaNFromPointCloud(*m_clouds[index], *m_clouds[index], indices);

			//run extra filters
			m_clouds[index] = downsampleCloud(m_clouds[index], LocalConfig::downsampleAmount);
			if(LocalConfig::removeOutliers) m_clouds[index] = removeOutliers(m_clouds[index], LocalConfig::deviations, LocalConfig::kSamples);
//			if(!LocalConfig::usingMultipleKinects) m_clouds[index] = clusterFilter(m_clouds[index], LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

			//translate cloud in z direction
			if(LocalConfig::translateCloud) {
				for(int i=0; i<m_clouds[index]->size(); ++i) {
					m_clouds[index]->at(i).z += LocalConfig::zOffset;
				}
			}

			//transform combined point cloud back to the first kinect's reference frame
			pcl::transformPointCloud(*m_clouds[index], *m_clouds[index], inverseTransform);

			//publish color image, depth image, and point cloud
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

			if(LocalConfig::isKinect2) {
				LocalConfig::maskSize = 100;
				LocalConfig::clusterTolerance = 0.1;
				LocalConfig::clusterMinSize = 200;
				LocalConfig::lowThreshold = 0.006;
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

	transforms.reserve(nCameras);
	scales.reserve(nCameras);
	alignments.reserve(nCameras);
	transforms_init.assign(nCameras, false);

	PreprocessorSegmentationNode tp(nh);
	tp.init(nh);

	ros::spin();
}
