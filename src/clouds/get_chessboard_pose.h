#include <vector>
//////////////////#include <cv.h>
#include <opencv/cv.h>
/////////#include <highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

//the transform brings points from the camera coordinate system to the reference (center of chess board) coordinate system
bool get_chessboard_pose(cv::Mat& image, int width_cb, int height_cb, double size, const Eigen::Matrix3f& cam_matrix, Eigen::Matrix4f& transform);
