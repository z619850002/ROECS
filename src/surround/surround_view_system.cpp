#include "../../include/surround/surround_view_system.h"
#include "utils.h"

using namespace std;





SurroundView::SurroundView(){
	//Generate ROI.
	//Determined optimized ROI.
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
	
	this->m_iROI_FL = cv::Rect(ROI_FL_x-50,ROI_FL_y,ROI_FL_w,ROI_FL_h+70);
	this->m_iROI_LB = cv::Rect(ROI_LB_x,ROI_LB_y-100,ROI_LB_w,ROI_LB_h+70);
	this->m_iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y-100,ROI_BR_w,ROI_BR_h+70);
	this->m_iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w,ROI_RF_h+70);

}



SurroundView::SurroundView(	Camera * pFrontCamera, Camera * pLeftCamera, 
					Camera * pBackCamera, Camera * pRightCamera):
	m_pFrontCamera(pFrontCamera), m_pLeftCamera(pLeftCamera),
	m_pBackCamera(pBackCamera), m_pRightCamera(pRightCamera)
{
	//Generate ROI.
	//Determined optimized ROI.
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
	
	this->m_iROI_FL = cv::Rect(ROI_FL_x-50,ROI_FL_y,ROI_FL_w,ROI_FL_h+70);
	this->m_iROI_LB = cv::Rect(ROI_LB_x,ROI_LB_y-100,ROI_LB_w,ROI_LB_h+70);
	this->m_iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y-100,ROI_BR_w,ROI_BR_h+70);
	this->m_iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w,ROI_RF_h+70);
	
}

bool SurroundView::GetUndistortedROI(int nIndex, int nCameraIndex, cv::Mat & mROI_Left, cv::Mat & mROI_Right,
										vector<int> & gROI_Left , vector<int> & gROI_Right){
	//Get the original image.
	vector<Frame *> gpFrames = {
		this->m_gDistortedPairs[nIndex].m_pFrontFrame,
		this->m_gDistortedPairs[nIndex].m_pLeftFrame,
		this->m_gDistortedPairs[nIndex].m_pBackFrame,
		this->m_gDistortedPairs[nIndex].m_pRightFrame
	};
	cv::Mat mOriginalImage;
	mOriginalImage = gpFrames[nCameraIndex]->m_mFisheyeImage;

	//Get the ROI and relative camera intrinsics.
	vector<cv::Rect> gRect = {	
		m_iROI_FL, m_iROI_RF,
		m_iROI_LB, m_iROI_FL,
		m_iROI_BR, m_iROI_LB,
		m_iROI_RF, m_iROI_BR
	};

	vector<Camera *> gCamera = {
		m_pFrontCamera,
		m_pLeftCamera,
		m_pBackCamera,
		m_pRightCamera
	};

	//Get 2 overlapping region and the intrinsics.
	cv::Rect iRectLeft = gRect[nCameraIndex*2];
	cv::Rect iRectRight = gRect[nCameraIndex*2+1];
	Camera * pCamera = gCamera[nCameraIndex];
	cv::Mat mK, mD;
	cv::eigen2cv(pCamera->m_mK, mK);
	cv::eigen2cv(pCamera->m_mD, mD);


	//Get 2 Rois
	CalculateROI(mOriginalImage, mROI_Left, mK, mD, pCamera->m_mT, this->m_mK_G,
				 iRectLeft.x, iRectLeft.y, iRectLeft.width, iRectLeft.height,
				 gROI_Left);


	CalculateROI(mOriginalImage, mROI_Right, mK, mD, pCamera->m_mT, this->m_mK_G,
				 iRectRight.x, iRectRight.y, iRectRight.width, iRectRight.height,
				 gROI_Right);
}



cv::Mat SurroundView::GenerateBirdsView(int nIndex, int nCamera, int nRows, int nCols){
	if (nIndex >= this->m_gDistortedPairs.size()){
		cout << "Pairs out of index, please check the index of the surround view inputed.";
	}

	SVPair iPair = this->m_gDistortedPairs[nIndex];

	//Get the bird's-eye view image.

	switch (nCamera) {
		//Front
		case 0: {
			cv::Mat mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 this->m_pFrontCamera->m_mT,
											 this->m_pFrontCamera->m_mK, 
											 this->m_pFrontCamera->m_mD,
											 this->m_mK_G, nRows, nCols);
			return mBirdsFront;
		}
		break;

		case 1: {

			cv::Mat mBirdsLeft = project_on_ground(  iPair.m_pLeftFrame->m_mFisheyeImage,
													 this->m_pLeftCamera->m_mT,
													 this->m_pLeftCamera->m_mK, 
													 this->m_pLeftCamera->m_mD,
													 this->m_mK_G, nRows, nCols);
			return mBirdsLeft;
		}
		break;

		case 2: {
			cv::Mat mBirdsBack = project_on_ground(  iPair.m_pBackFrame->m_mFisheyeImage,
													 this->m_pBackCamera->m_mT,
													 this->m_pBackCamera->m_mK, 
													 this->m_pBackCamera->m_mD,
													 this->m_mK_G, nRows, nCols);			
			return mBirdsBack;
		}
		break;

		case 3: {
			cv::Mat mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
													 this->m_pRightCamera->m_mT,
													 this->m_pRightCamera->m_mK, 
													 this->m_pRightCamera->m_mD,
													 this->m_mK_G, nRows, nCols);
			return mBirdsRight;
		}
		break;
	
	};

	cv::Mat mNullView;
	cout << "Wrong camera index." << endl;
	return mNullView;
}






cv::Mat SurroundView::GenerateSurroundView(int nIndex, int nRows, int nCols){
	if (nIndex >= this->m_gDistortedPairs.size()){
		cout << "Pairs out of index, please check the index of the surround view inputed.";
	}

	SVPair iPair = this->m_gDistortedPairs[nIndex];

	//Get the bird's-eye view image.


	cv::Mat mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 this->m_pFrontCamera->m_mT,
											 this->m_pFrontCamera->m_mK, 
											 this->m_pFrontCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	cv::Mat mBirdsLeft = project_on_ground(  iPair.m_pLeftFrame->m_mFisheyeImage,
											 this->m_pLeftCamera->m_mT,
											 this->m_pLeftCamera->m_mK, 
											 this->m_pLeftCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	cv::Mat mBirdsBack = project_on_ground(  iPair.m_pBackFrame->m_mFisheyeImage,
											 this->m_pBackCamera->m_mT,
											 this->m_pBackCamera->m_mK, 
											 this->m_pBackCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	cv::Mat mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
											 this->m_pRightCamera->m_mT,
											 this->m_pRightCamera->m_mK, 
											 this->m_pRightCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	
	
	//Stitch the bird's-eye view image to get the surround-view image.
	cv::Mat mSurroundView = generate_surround_view(mBirdsFront, mBirdsLeft, mBirdsBack, mBirdsRight, nRows, nCols);
	return mSurroundView;
}


bool SurroundView::InitK_G(int nRows, int nCols, float nDx, float nDy){
	cv::Mat mK_G = cv::Mat::zeros(3,3,CV_64FC1);
	mK_G.at<double>(0,0) = 1/nDx;
	mK_G.at<double>(1,1) = -1/nDy;
	mK_G.at<double>(0,2) = nCols/2;
	mK_G.at<double>(1,2) = nRows/2;
	mK_G.at<double>(2,2) =   1.0;

	this->m_mK_G = mK_G;
	return true;
}


bool SurroundView::BindImagePairs(vector<SVPair> gDistortedPairs){
	this->m_gDistortedPairs = gDistortedPairs;
}