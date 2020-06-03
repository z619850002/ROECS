#ifndef MY_PIXEL_SELECTION_H_
#define MY_PIXEL_SELECTION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>

using namespace std;




class PixelSelection
{
public:
	//Destructor.
	PixelSelection();
	int m_nFlag;
	
	vector<cv::Point2d> GetPixels(cv::Mat & mROI);


	vector<cv::Point2d> GetPixels(cv::Mat & mROI , int nCameraIndex , int nChooseCameraIndex);

	vector<cv::Point2d> GetCullingPixels(cv::Mat & mROI,
 										 cv::Mat & mCurrentImage, 
 										 cv::Mat & mNextImage,
										 cv::Rect iROI);


private:
	vector<cv::Point2d> GetDensePixels(cv::Mat & mROI);

	vector<cv::Point2d> GetSparsePixels(cv::Mat & mROI);
	
};



#endif