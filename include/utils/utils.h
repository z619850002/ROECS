#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace std;


//Change the image cooridnate to the normazed coordinate.
cv::Point2d Pixel2Cam ( const cv::Point2d& p, const cv::Mat& K );






#endif