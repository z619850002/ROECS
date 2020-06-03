#include "../../include/selection/pixel_selection.h"
#include "../../include/selection/selection_util.h"
#include <opencv2/opencv.hpp>
#include <time.h>


using namespace std;



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
	cv::Sobel(mROI, mGradients, CV_64FC1, 3, 3, 7);
	cv::Scalar     iMean;  
    cv::Scalar     iDev;
	cv::meanStdDev(mGradients , iMean , iDev);
	double nMean = iMean.val[0];
	double nDev = iDev.val[0];

	vector<cv::Point2d> gPoints;
	gPoints.reserve(mROI.size().width * mROI.size().height);
	for (int u=0;u<mROI.size().width;u++){
		for (int v=0;v<mROI.size().height;v++){
			if (mGradients.at<double>(v , u) > nMean + nDev/2 && mGradients.at<double>(v , u) > 70){
				gPoints.push_back(cv::Point2d(u , v));
			}
		}
	}


	return gPoints;
}



vector<cv::Point2d> PixelSelection::GetCullingPixels(cv::Mat & mROI,
 													 cv::Mat & mCurrentImage, 
 													 cv::Mat & mNextImage,
													 cv::Rect iROI){
	time_t tBegin, tEnd;
	tBegin = clock();
	vector<cv::Point2d> mNullVector;
	if (this->m_nFlag == 0){
		mNullVector =  GetDensePixels(mROI);	
	}else{
		mNullVector =  GetSparsePixels(mROI);
	}

	cout << "Point size" << mNullVector.size() << endl;

	tEnd = clock();
	cout << "Sparse cost " << (double)(tEnd - tBegin) / (double)(CLOCKS_PER_SEC) << endl;

	//Homography estimation.
	tBegin = clock();
	vector<Eigen::Vector2d> gCurrentPoints, gNextPoints;
	GeneratePointsPair(mCurrentImage, mNextImage, gCurrentPoints, gNextPoints);
	tEnd = clock();
	cout << "Pair cost " << (double)(tEnd - tBegin) / (double)(CLOCKS_PER_SEC) << endl;
	
	//Check the pair size.
	if (gCurrentPoints.size() < 8){
		return mNullVector;
	}

	//Check the diversity of all points.
	vector<double> gX, gY;
	double nAverageX, nAverageY;
	for (auto item : gCurrentPoints){
		gX.push_back(item(0));
		gY.push_back(item(1));
		nAverageX += item(0);
		nAverageY += item(1);
	}

	nAverageX /= gCurrentPoints.size();
	nAverageY /= gCurrentPoints.size();

	double nStdX, nStdY;

	for (auto item : gCurrentPoints){
		nStdX += (item(0) - nAverageX) * (item(0) - nAverageX);
		nStdY += (item(1) - nAverageY) * (item(1) - nAverageY);
	}

	nStdX /= gCurrentPoints.size();
	nStdY /= gCurrentPoints.size();
	
	if (sqrt(nStdX * nStdX + nStdY * nStdY) <=30000){
		return mNullVector;
	} 



	cv::Mat mHomography;
	//mHomography * point1 = point2
	tBegin = clock();
	OptimizeHomographyPose(gCurrentPoints, gNextPoints, mHomography);
	tEnd = clock();
	cout << "Homography cost " << (double)(tEnd - tBegin) / (double)(CLOCKS_PER_SEC) << endl;


	//Check the homography result.
	double nDet = cv::determinant(mHomography.rowRange(0, 2).colRange(0 , 2));
	if (sqrt((nDet-1) * (nDet - 1)) > 0.2){
		return mNullVector;
	}


	//Generate image.
	cv::Mat mGeneratedImage;

	tBegin = clock();
	CopyImage(mNextImage, mGeneratedImage, mHomography);
	tEnd = clock();
	cout << "Copy cost " << (double)(tEnd - tBegin) / (double)(CLOCKS_PER_SEC) << endl;


	cv::Mat mROI_Original, mROI_Generated;
	mROI_Original = mCurrentImage(iROI);
	mROI_Generated = mGeneratedImage(iROI);

	tBegin = clock();
	
	mNullVector = Culling(mNullVector, mROI_Original, mROI_Generated);
	
	tEnd = clock();
	cout << "Culling cost " << (double)(tEnd - tBegin) / (double)(CLOCKS_PER_SEC) << endl;

	return mNullVector;
}






vector<cv::Point2d> PixelSelection::GetPixels(cv::Mat & mROI, int nCameraIndex , int nChooseCameraIndex){
	vector<cv::Point2d> mNullVector;
	if (this->m_nFlag == 0){
		mNullVector =  GetDensePixels(mROI);	
	}else{
		mNullVector =  GetSparsePixels(mROI);
	}

	cv::Mat mROI_1, mROI_2;
	mROI_1 = mROI.clone();
	mROI_2 = mROI.clone();

	int ROI_FL_x = 200;
	int ROI_FL_y = 0;
	int ROI_FL_w = 200;
	int ROI_FL_h = 200;
	
	int ROI_LB_x = 200;
	int ROI_LB_y = 800;
	int ROI_LB_w = 200;
	int ROI_LB_h = 200;
	
	int ROI_BR_x = 650;
	int ROI_BR_y = 800;
	int ROI_BR_w = 200;
	int ROI_BR_h = 200;
	
	int ROI_RF_x = 650;
	int ROI_RF_y = 0;
	int ROI_RF_w = 200;
	int ROI_RF_h = 200;
	
	cv::Rect iROI_FL = cv::Rect(ROI_FL_x-50,ROI_FL_y,ROI_FL_w,ROI_FL_h+70);
	cv::Rect iROI_LB = cv::Rect(ROI_LB_x,ROI_LB_y-100 - 700,ROI_LB_w,ROI_LB_h+70);
	cv::Rect iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y-100 - 700,ROI_BR_w + 150,ROI_BR_h+70);
	cv::Rect iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w + 100,ROI_RF_h+70);


	cv::Mat mFrontGenerated = cv::imread("../culling/front_generated.jpg");
	cv::Mat mBackGenerated = cv::imread("../culling/back_generated.jpg");

	cv::Mat mFrontFrom = cv::imread("../culling/front_from.jpg");
	cv::Mat mBackFrom = cv::imread("../culling/back_from.jpg");


	// cv::imshow("ROI", mFrontFrom);
	// cv::waitKey(0);
	// cv::imshow("ROI", mFrontGenerated);
	// cv::waitKey(0);



	cout << "Camera Index is " << nCameraIndex << endl;
	cout << "nChoose index is " << nChooseCameraIndex << endl;
	cv::Mat mGeneratedROI , mFromROI;
	if (nCameraIndex == 1 && nChooseCameraIndex == 0){
		mGeneratedROI = mFrontGenerated(iROI_FL);
		mFromROI = mFrontFrom(iROI_FL);
	}else if (nCameraIndex == 1 && nChooseCameraIndex == 2){
		mGeneratedROI = mBackGenerated(iROI_LB);
		mFromROI = mBackFrom(iROI_LB);
	}else if (nCameraIndex == 3 && nChooseCameraIndex == 0){
		mGeneratedROI = mFrontGenerated(iROI_RF);
		mFromROI = mFrontFrom(iROI_RF);
	}else if (nCameraIndex == 3 && nChooseCameraIndex == 2){
		mGeneratedROI = mBackGenerated(iROI_BR);
		mFromROI = mBackFrom(iROI_BR);
	}

	cv::imshow("ROI", mGeneratedROI);
	cv::waitKey(0);
	cv::imshow("ROI", mFromROI);
	cv::waitKey(0);

	mNullVector = Culling(mNullVector, mGeneratedROI, mFromROI);

	return mNullVector;
}
