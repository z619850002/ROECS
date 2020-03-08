#include "../../include/selection/pixel_selection.h"
#include <opencv2/opencv.hpp>


//Constructor.
PixelSelection::PixelSelection(){
	//flag = 0   dense
	//flag = 1   sparse
	m_nFlag = 1;
}



vector<cv::Point2d> PixelSelection::GetPixels(cv::Mat & mROI){
	vector<cv::Point2d> mNullVector;
	if (this->m_nFlag == 0){
		return GetDensePixels(mROI);	
	}else{
		return GetSparsePixels(mROI);
	}
	return mNullVector;
}


vector<cv::Point2d> PixelSelection::GetDensePixels(cv::Mat & mROI){
	vector<cv::Point2d> gPoints;
	gPoints.reserve(mROI.size().width * mROI.size().height);
	for (int u=0;u<mROI.size().width;u++){
		for (int v=0;v<mROI.size().height;v++){
			gPoints.push_back(cv::Point2d(u , v));
		}
	}
	return gPoints;
}



vector<cv::Point2d> PixelSelection::GetSparsePixels(cv::Mat & mROI){

	cv::Mat mGradients;
	cv::Sobel(mROI, mGradients, CV_64FC1, 1, 0, 7);
	cv::Scalar     iMean;  
    cv::Scalar     iDev;
	cv::meanStdDev(mGradients , iMean , iDev);
	double nMean = iMean.val[0];
	double nDev = iDev.val[0];

	vector<cv::Point2d> gPoints;
	gPoints.reserve(mROI.size().width * mROI.size().height);
	for (int u=0;u<mROI.size().width;u++){
		for (int v=0;v<mROI.size().height;v++){
			if (mGradients.at<double>(v , u) > nMean + nDev/2){
				gPoints.push_back(cv::Point2d(u , v));
			}
		}
	}


	return gPoints;
}