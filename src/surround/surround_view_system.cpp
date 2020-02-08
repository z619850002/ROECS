#include "../../include/surround/surround_view_system.h"
#include "utils.h"
#include "../../include/optimizer/direct_unary_edge.h"
#include "../../include/optimizer/surround_optimizer.h"
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

	// //Initialize optimizer.
	// this->m_pOptimizer = new SurroundOptimizer(
	// 	this->m_pFrontCamera,
	// 	this->m_pLeftCamera,
	// 	this->m_pBackCamera,
	// 	this->m_pRightCamera);

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


	this->m_pOptimizer = new SurroundOptimizer(
		this->m_pFrontCamera,
		this->m_pLeftCamera,
		this->m_pBackCamera,
		this->m_pRightCamera);
	
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
		m_iROI_RF, m_iROI_FL,
		m_iROI_FL, m_iROI_LB,
		m_iROI_LB, m_iROI_BR,
		m_iROI_BR, m_iROI_RF
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


// Add edges of the optimization in one frame for one camera.
// nIndex : the index of the frame chosen in optimization
// nCameraIndex : the camera want to be optimized
bool SurroundView::AddEdge(int nIndex, int nCameraIndex,
							vector<cv::Mat> & gSurroundViews,
							cv::Mat & mGrayROI_Right,
							cv::Mat & mGrayROI_Left){
	cv::Mat mSurroundView_Right, mSurroundView_Left, mSurroundView;
	vector<int> gRightIndex = {3,0,1,2};
	vector<int> gLeftIndex = {1,2,3,0};
	int nRightIndex = gRightIndex[nCameraIndex];
	int nLeftIndex = gLeftIndex[nCameraIndex];

	//Used to transfer surround-view coordinate to ground coordinate
	cv::Mat mK_G_inv = this->m_mK_G.inv();
	cv::Mat mK_G_Augment;
	cv::vconcat( mK_G_inv.rowRange(0 , 2) , cv::Mat::zeros(1 , 3 , CV_64FC1) , mK_G_Augment);
	cv::vconcat( mK_G_Augment , mK_G_inv.rowRange(2 , 3) , mK_G_Augment);

	//Generate ROI on original images.
	cv::Mat mOriginROI_Right, mOriginROI_Left;
	vector<int> gOriginROI_Right, gOriginROI_Left;
	//Get the ROI on original image.
	GetUndistortedROI(	nIndex, nCameraIndex, mOriginROI_Right, mOriginROI_Left,
					 	gOriginROI_Right, gOriginROI_Left);

	//Construct gray image.
    // cv::Mat mGrayROI_Right , mGrayROI_Left;
    cv::cvtColor(mOriginROI_Right,mGrayROI_Right,cv::COLOR_BGR2GRAY);
    cv::cvtColor(mOriginROI_Left, mGrayROI_Left, cv::COLOR_BGR2GRAY);


    //Add edges.
	//The depth is fixed now.
	//First measurement ROI
    //Get 2 ROI.
    vector<cv::Rect> gLeftROIs = {m_iROI_RF, m_iROI_FL, m_iROI_LB, m_iROI_BR};
	vector<cv::Rect> gRightROIs = {m_iROI_FL, m_iROI_LB, m_iROI_BR, m_iROI_RF};
	cv::Rect iLeftROI = gLeftROIs[nLeftIndex];
	cv::Rect iRightROI = gRightROIs[nRightIndex];

	cv::Mat mMeasurementROI_Right = gSurroundViews[nRightIndex](iRightROI);
	cv::Mat mMeasurementROI_Left = gSurroundViews[nLeftIndex](iLeftROI);
	cv::Mat mMeasurementGray_Right , mMeasurementGray_Left;
    cv::cvtColor(mMeasurementROI_Right, mMeasurementGray_Right, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mMeasurementROI_Left, mMeasurementGray_Left, cv::COLOR_BGR2GRAY);
	mMeasurementGray_Right.convertTo(mMeasurementGray_Right, CV_64FC1);
	mMeasurementGray_Left.convertTo(mMeasurementGray_Left, CV_64FC1);

	//The ROI on birds-eye view of this camera.
	//Used in computing the coef.
	//Get the ROI.
	cv::Mat mBirdseyeROI_Right = gSurroundViews[nCameraIndex](iRightROI);
	cv::Mat mBirdseyeROI_Left = gSurroundViews[nCameraIndex](iLeftROI);

	//Convert to grayscale.
	cv::Mat mBirdseyeGray_Right, mBirdseyeGray_Left;
    cv::cvtColor(mBirdseyeROI_Right, mBirdseyeGray_Right, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mBirdseyeROI_Left, mBirdseyeGray_Left, cv::COLOR_BGR2GRAY);
	mBirdseyeGray_Right.convertTo(mBirdseyeGray_Right, CV_64FC1);
	mBirdseyeGray_Left.convertTo(mBirdseyeGray_Left, CV_64FC1);

	//Get the coef to eliminate the affect of exposure time.
	double nCoef_Right = cv::mean(mBirdseyeGray_Right).val[0]/cv::mean(mMeasurementGray_Right).val[0];
	double nCoef_Left = cv::mean(mBirdseyeGray_Left).val[0]/cv::mean(mMeasurementGray_Left).val[0];

	//Add edges in the right region.
	for (int u=0;u<iRightROI.width;u++){
		for (int v=0;v<iRightROI.height;v++){
			//Get the surround-view coordinate of the point.
			int nU = u + iRightROI.x;
			int nV = v + iRightROI.y;
			cv::Mat mp_surround = (cv::Mat_<double>(3 , 1) << nU, nV, 1);
			//Convert surround-view coordinate to ground coordinate.
			cv::Mat mP_G = mK_G_Augment * mp_surround;


            Eigen::Vector3d mPoint3d(mP_G.at<double>(0 , 0),
            						 mP_G.at<double>(1 , 0),
            						 mP_G.at<double>(2 , 0));


            double x = mPoint3d[0];
            double y = mPoint3d[1];
            double z = mPoint3d[2];

            vector<Camera *> gpCameras = {
            	this->m_pFrontCamera,
            	this->m_pLeftCamera,
            	this->m_pBackCamera,
            	this->m_pRightCamera
            };


            double nMeasurement = mMeasurementGray_Right.at<double>(v , u) * nCoef_Right;
            this->m_pOptimizer->AddEdge(
            	mPoint3d,
            	nCameraIndex,
            	gOriginROI_Right,
            	nMeasurement,
            	&mGrayROI_Right);
		}		
	}

	//Add edges in the left region.
	for (int u=0;u<iLeftROI.width;u++){
		for (int v=0;v<iLeftROI.height;v++){
			//Get the surround-view coordinate of the point.
			int nU = u + iLeftROI.x;
			int nV = v + iLeftROI.y;
			cv::Mat mp_surround = (cv::Mat_<double>(3 , 1) << nU, nV, 1);
			//Convert surround-view coordinate to ground coordinate.
			cv::Mat mP_G = mK_G_Augment * mp_surround;


            Eigen::Vector3d mPoint3d(mP_G.at<double>(0 , 0),
            						 mP_G.at<double>(1 , 0),
            						 mP_G.at<double>(2 , 0));

            double nMeasurement = mMeasurementGray_Left.at<double>(v , u) * nCoef_Left;
            this->m_pOptimizer->AddEdge(
            	mPoint3d,
            	nCameraIndex,
            	gOriginROI_Left,
            	nMeasurement,
            	&mGrayROI_Left);
		}		
	}	
}




bool SurroundView::OptimizePoseWithOneFrame(int nIndex){
	//Generate birds-eye view image.
	//Useless.
	cv::Mat mSurroundView_Front = GenerateBirdsView(nIndex, 0,  1000, 1000);
	cv::Mat mSurroundView_Left = GenerateBirdsView(nIndex, 1,  1000, 1000);
	cv::Mat mSurroundView_Back = GenerateBirdsView(nIndex, 2,  1000, 1000);
	cv::Mat mSurroundView_Right = GenerateBirdsView(nIndex, 3,  1000, 1000);

	vector<cv::Mat> gSurroundViews = {
		mSurroundView_Front,
		mSurroundView_Left,
		mSurroundView_Back,
		mSurroundView_Right
	};

	cv::Mat mGrayROI_Right,mGrayROI_Left;
	cv::Mat mGrayROI_Right2,mGrayROI_Left2;
	this->AddEdge(nIndex, 1, gSurroundViews, mGrayROI_Right, mGrayROI_Left);
	this->AddEdge(nIndex, 3, gSurroundViews, mGrayROI_Right2, mGrayROI_Left2);
	// cout << "mGrayROI_Right size " << endl << mGrayROI_Right.rows << "*" << mGrayROI_Right.cols << endl; 

	this->m_pOptimizer->Optimize();
}


bool SurroundView::OptimizeWithMultiFrame(vector<int> gIndices){
	vector<cv::Mat> gGrayROI_Right, gGrayROI_Left;
	vector<cv::Mat> gGrayROI_Right2, gGrayROI_Left2;
	
	for (int i=0;i<gIndices.size();i++){
		cv::Mat mGrayROI_Right,mGrayROI_Left;
		cv::Mat mGrayROI_Right2,mGrayROI_Left2;
		gGrayROI_Right.push_back(mGrayROI_Right);
		gGrayROI_Right2.push_back(mGrayROI_Right2);
		gGrayROI_Left.push_back(mGrayROI_Left);
		gGrayROI_Left2.push_back(mGrayROI_Left2);
	}

	for (int i=0;i<gIndices.size();i++){
		int nIndex = gIndices[i];
		cv::Mat mSurroundView_Front = GenerateBirdsView(nIndex, 0,  1000, 1000);
		cv::Mat mSurroundView_Left = GenerateBirdsView(nIndex, 1,  1000, 1000);
		cv::Mat mSurroundView_Back = GenerateBirdsView(nIndex, 2,  1000, 1000);
		cv::Mat mSurroundView_Right = GenerateBirdsView(nIndex, 3,  1000, 1000);

		vector<cv::Mat> gSurroundViews = {
			mSurroundView_Front,
			mSurroundView_Left,
			mSurroundView_Back,
			mSurroundView_Right
		};
		this->AddEdge(nIndex, 1, gSurroundViews, gGrayROI_Right[i], gGrayROI_Left[i]);
		this->AddEdge(nIndex, 3, gSurroundViews, gGrayROI_Right2[i], gGrayROI_Left2[i]);
	}
	// for (auto nIndex : gIndices){
		//Generate birds-eye view image.
		
	// }
	this->m_pOptimizer->Optimize();
}


