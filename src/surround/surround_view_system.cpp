#include "../../include/surround/surround_view_system.h"
#include "surround_utils.h"

using namespace std;

cv::Mat DrawPoints(cv::Mat mImage , vector<cv::Point2d> gPoints){
	cv::Mat mImageDraw = mImage.clone();

	int nWidth = mImageDraw.size().width;
	int nHeight = mImageDraw.size().height;

	for (auto iOriginPoint : gPoints){
		for (int i=-2;i<=2;i++){
			for (int j=-2;j<=2;j++){
				if (iOriginPoint.x + i >=0 && iOriginPoint.x + i <nWidth && 
					iOriginPoint.y + j >=0 && iOriginPoint.y + j < nHeight){
					cv::Point2d iPoint(iOriginPoint.x + i , iOriginPoint.y + j);
					// double nDistance = sqrt( (iPoint.x / (double)nWidth) * (iPoint.x / (double)nWidth) +
				 // 						 (iPoint.y / (double)nHeight) * (iPoint.y / (double)nHeight))/sqrt(2);
				double nDistance = sqrt( (iOriginPoint.x / (double)nWidth) * (iOriginPoint.x / (double)nWidth) +
				 						 ((nHeight - iOriginPoint.y) / (double)nHeight) * ((nHeight - iOriginPoint.y) / (double)nHeight))/sqrt(2) ;

				double nDistance2 = sqrt( ((iOriginPoint.x - nWidth/2) / (double)(nWidth/2)) * ((iOriginPoint.x- nWidth/2) / (double)(nWidth/2)) +
				 						 ((nHeight/2 - iOriginPoint.y) / (double)(nHeight/2)) * ((nHeight/2 - iOriginPoint.y) / (double)(nHeight/2)))/sqrt(2);

					// if (iOriginPoint.x < 10 && iOriginPoint.y < 10){
					// 	cout << "Distance: " << nDistance2 << endl;
					
					// }
					// if (nDistance >=2){
					// 	nDistance = nDistance-2;
					// } else if (nDistance >= 1){
					// 	nDistance = 2 - nDistance;
					// }
					int nBlue = (int)(255.0 * nDistance);
					int nRed = (int)(255.0 * (1-nDistance));
					int nGreen = (int)(255.0 * (1-nDistance2)) ;
					mImageDraw.at<cv::Vec3b>(iPoint.y , iPoint.x) = cv::Vec3b(nBlue , nGreen , nRed);					
				}
				
			}
		}

	}
	return mImageDraw;

}





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
	this->m_iROI_LB = cv::Rect(ROI_LB_x - 50,ROI_LB_y-100,ROI_LB_w,ROI_LB_h+70);
	this->m_iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y-100,ROI_BR_w,ROI_BR_h+70);
	this->m_iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w + 100,ROI_RF_h+70);


	// this->m_iROI_FL = cv::Rect(ROI_FL_x,ROI_FL_y,ROI_FL_w,ROI_FL_h);
	// this->m_iROI_LB = cv::Rect(ROI_LB_x,ROI_LB_y,ROI_LB_w,ROI_LB_h);
	// this->m_iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y,ROI_BR_w,ROI_BR_h);
	// this->m_iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w,ROI_RF_h);

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
	this->m_iROI_LB = cv::Rect(ROI_LB_x - 50,ROI_LB_y-100,ROI_LB_w,ROI_LB_h+70);
	this->m_iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y-100,ROI_BR_w,ROI_BR_h+70);
	this->m_iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w + 100,ROI_RF_h+70);



	this->m_pOptimizer = new SurroundOptimizer(
		this->m_pFrontCamera,
		this->m_pLeftCamera,
		this->m_pBackCamera,
		this->m_pRightCamera);
	
}

bool SurroundView::GetBirdseyeROI(	int nIndex, int nPosIndex,
							 		cv::Mat & mROI_Left, cv::Mat & mROI_Right){
	//Pos index = 0, FL,
	//Pos index = 1, LB,
	//Pos index = 2, BR,
	//Pos index = 3, RF.

	vector<int> gLeftIndex = {1 , 2 , 3 , 0};
	vector<int> gRightIndex = {0 , 1 , 2 , 3};

	cv::Mat mLeftBirdsEye = this->GenerateBirdsView(nIndex, gLeftIndex[nPosIndex], 1000, 1000);
	cv::Mat mRightBirdsEye = this->GenerateBirdsView(nIndex, gLeftIndex[nPosIndex], 1000, 1000);

	// mROI_Left = mLeftBirdsEye
	vector<cv::Rect> gRect = {	
		m_iROI_FL, m_iROI_LB, m_iROI_BR, m_iROI_RF
	};

	mROI_Left = mLeftBirdsEye(gRect[nPosIndex]);
	mROI_Right = mRightBirdsEye(gRect[nPosIndex]);

	return true;
}


bool SurroundView::GetCoupleROI(int nIndex, int nCameraIndex_Left, int nCameraIndex_Right,
								cv::Mat & mROI_Left, cv::Mat & mROI_Right,
								vector<int> & gROI_Left, vector<int> & gROI_Right){
	//Get the original image.
	vector<Frame *> gpFrames = {
		this->m_gDistortedPairs[nIndex].m_pFrontFrame,
		this->m_gDistortedPairs[nIndex].m_pLeftFrame,
		this->m_gDistortedPairs[nIndex].m_pBackFrame,
		this->m_gDistortedPairs[nIndex].m_pRightFrame
	};
	cv::Mat mOriginalImage_Left, mOriginalImage_Right;
	mOriginalImage_Left = gpFrames[nCameraIndex_Left]->m_mFisheyeImage;
	mOriginalImage_Right = gpFrames[nCameraIndex_Right]->m_mFisheyeImage;

	//Get the ROI and relative camera intrinsics.
	vector<cv::Rect> gRect = {	
		m_iROI_FL,
		m_iROI_LB,
		m_iROI_BR,
		m_iROI_RF
	};

	vector<Camera *> gCamera = {
		m_pFrontCamera,
		m_pLeftCamera,
		m_pBackCamera,
		m_pRightCamera
	};

	//Get 2 overlapping region and the intrinsics.
	cv::Rect iRect = gRect[nCameraIndex_Right];
	Camera * pCamera_Left = gCamera[nCameraIndex_Left];
	Camera * pCamera_Right = gCamera[nCameraIndex_Right];


	cv::Mat mK_Left, mD_Left;
	cv::Mat mK_Right, mD_Right;
	cv::eigen2cv(pCamera_Left->m_mK, mK_Left);
	cv::eigen2cv(pCamera_Left->m_mD, mD_Left);
	cv::eigen2cv(pCamera_Right->m_mK, mK_Right);
	cv::eigen2cv(pCamera_Right->m_mD, mD_Right);


	//Get 2 Rois
	CalculateROI(mOriginalImage_Left, mROI_Left, mK_Left, mD_Left,
			 	 pCamera_Left->m_mT, this->m_mK_G,
				 iRect.x, iRect.y, iRect.width, iRect.height,
				 gROI_Left);


	CalculateROI(mOriginalImage_Right, mROI_Right, mK_Right, mD_Right,
	 			 pCamera_Right->m_mT, this->m_mK_G,
				 iRect.x, iRect.y, iRect.width, iRect.height,
				 gROI_Right);
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

	PixelSelection iSelection;
	vector<cv::Point2d> gPointsRight = iSelection.GetPixels(mMeasurementGray_Right);
	vector<cv::Point2d> gPointsLeft = iSelection.GetPixels(mMeasurementGray_Left);
	

	//Add edges in the right region.
	for (cv::Point2d iPoint2d : gPointsRight){
		int u = iPoint2d.x , v = iPoint2d.y;
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
            // this->m_pOptimizer->AddEdge(
            // 	mPoint3d,
            // 	nCameraIndex,
            // 	gOriginROI_Right,
            // 	nMeasurement,
            // 	&mGrayROI_Right);

            this->m_pOptimizer->AddBinaryEdge(
            	mPoint3d,
            	nCameraIndex,
            	gOriginROI_Right,
            	nMeasurement,
            	&mGrayROI_Right);	

	}

	//Add edges in the left region.
	for (cv::Point2d iPoint2d : gPointsLeft){
			int u = iPoint2d.x , v = iPoint2d.y;
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
            // this->m_pOptimizer->AddEdge(
            // 	mPoint3d,
            // 	nCameraIndex,
            // 	gOriginROI_Left,
            // 	nMeasurement,
            // 	&mGrayROI_Left);
            
            this->m_pOptimizer->AddBinaryEdge(
            	mPoint3d,
            	nCameraIndex,
            	gOriginROI_Left,
            	nMeasurement,
            	&mGrayROI_Left);		
	}	
}

// void CalculateTransormation(cv::Mat mROI_1, cv::Mat mROI_2 , double & nCoef  , double & nTranslation){
// 	int nWidth = mROI_1.size().width;
// 	int nHeight = mROI_1.size().height;
// 	cv::Mat mA, mb;
// 	bool bFirst = true;
// 	mA = cv::Mat::zeros(nWidth * nHeight , 2, CV_64FC1);
// 	mb = cv::Mat::zeros(nWidth * nHeight , 1, CV_64FC1);

// 	int nSize = 0;
// 	for (int u=0;u<nWidth;u++){
// 		for (int v=0;v<nHeight;v++){
// 			if (fabs(mROI_1.at<double>(v , u) - mROI_2.at<double>(v , u)) <30){
// 				mA.at<double>(nSize , 0) = mROI_1.at<double>(v , u);
// 				mA.at<double>(nSize , 1) = 1;
// 				mb.at<double>(nSize , 0) = mROI_2.at<double>(v , u);
// 				nSize++;
// 			}
// 		}
// 	}
// 	cout << "nSize is " << nSize << endl;
// 	mA = mA.rowRange(0, nSize);
// 	mb = mb.rowRange(0, nSize);
// 	cout << "Res is " << (mA.t() * mA).inv() * mA.t() * mb << endl;	
// }



bool SurroundView::AddCoupleEdges(int nIndex, 
					int nCameraIndex_1, int nCameraIndex_2,
					vector<cv::Mat> & gSurroundViews,
					cv::Mat & mGrayROI_1,
					cv::Mat & mGrayROI_2){
	//Check if 2 cameras are neighbourhood.
	int nCheck = nCameraIndex_1-nCameraIndex_2;
	if (nCheck*nCheck == 4){
		cout << "2 cameras are not neighbourhood!" << endl;
		return false;
	}

	//The projection is from 1 to 2.
	cv::Mat mSurroundView_1, mSurroundView_2;
	

	//Used to transfer surround-view coordinate to ground coordinate
	cv::Mat mK_G_inv = this->m_mK_G.inv();
	cv::Mat mK_G_Augment;
	cv::vconcat( mK_G_inv.rowRange(0 , 2) , cv::Mat::zeros(1 , 3 , CV_64FC1) , mK_G_Augment);
	cv::vconcat( mK_G_Augment , mK_G_inv.rowRange(2 , 3) , mK_G_Augment);

	//Generate ROI on original images.
	cv::Mat mOriginROI_1, mOriginROI_2;
	vector<int> gOriginROI_1, gOriginROI_2;
	//Get the ROI on original image and on birds-eye image.
	vector<cv::Rect> gBirdROIs = {m_iROI_FL, m_iROI_LB, m_iROI_BR, m_iROI_RF};
	//Use the right index to get the ROI.
	cv::Rect iBirdsROI;
	//Get the ROI.
	if (nCameraIndex_1 - nCameraIndex_2 == 1 ||
		nCameraIndex_1 - nCameraIndex_2 == -3 ){
		//Left is 1, right is 2.
		// index_1 = 1 , 2 , 3 , 0
		// index_2 = 0 , 1 , 2 , 3
		GetCoupleROI(nIndex, nCameraIndex_1, nCameraIndex_2,
					 mOriginROI_1, mOriginROI_2,
				 	 gOriginROI_1, gOriginROI_2);
		iBirdsROI = gBirdROIs[nCameraIndex_2];
	}else if (nCameraIndex_1 - nCameraIndex_2 == -1 ||
			  nCameraIndex_1 - nCameraIndex_2 == 3 ){
		//Left is 2, right is 1.
		// index_1 = 0 , 1 , 2 , 3
		// index_2 = 1 , 2 , 3 , 0
		GetCoupleROI(nIndex, nCameraIndex_2, nCameraIndex_1,
					 mOriginROI_2, mOriginROI_1,
				 	 gOriginROI_2, gOriginROI_1);
		iBirdsROI = gBirdROIs[nCameraIndex_1];
	}else{
		cout << "Wrong ROI pairs!" << endl;
		return false;
	}
	
	//Construct gray image.
    cv::cvtColor(mOriginROI_1,mGrayROI_1,cv::COLOR_BGR2GRAY);
    cv::cvtColor(mOriginROI_2, mGrayROI_2, cv::COLOR_BGR2GRAY);

	//The ROI on birds-eye view of this camera.
	//Used in computing the coef.
	//Get the ROI.
	cv::Mat mBirdseyeROI_1 = gSurroundViews[nCameraIndex_1](iBirdsROI);
	cv::Mat mBirdseyeROI_2 = gSurroundViews[nCameraIndex_2](iBirdsROI);

	//Convert to grayscale.
	cv::Mat mBirdseyeGray_1, mBirdseyeGray_2;
    cv::cvtColor(mBirdseyeROI_1, mBirdseyeGray_1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mBirdseyeROI_2, mBirdseyeGray_2, cv::COLOR_BGR2GRAY);
	mBirdseyeGray_1.convertTo(mBirdseyeGray_1, CV_64FC1);
	mBirdseyeGray_2.convertTo(mBirdseyeGray_2, CV_64FC1);

	//Get the coef to eliminate the affect of exposure time.
	double nCoef = cv::mean(mBirdseyeGray_2).val[0]/cv::mean(mBirdseyeGray_1).val[0];
	// double nCoe , nB;
	// CalculateTransormation(mBirdseyeGray_1, mBirdseyeGray_2, nCoe, nB);
	// cout << "Coef is " << nCoef << endl;

	PixelSelection iSelection;
	// vector<cv::Point2d> gPoints = iSelection.GetPixels(mBirdseyeGray_1 , nCameraIndex_2, nCameraIndex_1);
	vector<cv::Point2d> gPoints = iSelection.GetPixels(mBirdseyeGray_1);
	
	cout << "Edge size " << gPoints.size() << endl;
	// if (gPoints.size() < 2000){
	// 	return false;
	// }
	//Add edges in the right region.
	for (cv::Point2d iPoint2d : gPoints){
		int u = iPoint2d.x , v = iPoint2d.y;
		//Get the surround-view coordinate of the point.
		int nU = u + iBirdsROI.x;
		int nV = v + iBirdsROI.y;
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


        double nMeasurement = mBirdseyeGray_1.at<double>(v , u) * nCoef;
        //From 1 to 2
        // this->m_pOptimizer->AddEdge(
        // 	mPoint3d,
        // 	nCameraIndex_2,
        // 	gOriginROI_2,
        // 	nMeasurement,
        // 	&mGrayROI_2);

        // this->m_pOptimizer->AddBinaryEdge(
        // 	mPoint3d,
        // 	nCameraIndex_2,
        // 	gOriginROI_2,
        // 	nMeasurement,
        // 	&mGrayROI_2);

        // this->m_pOptimizer->AddFixedBinaryEdge(
        // 	mPoint3d, 
        // 	nCameraIndex_1, 
        // 	gOriginROI_1, 
        // 	nMeasurement/nCoef, 
        // 	&mGrayROI_1, 
        // 	nCameraIndex_2, 
        // 	gOriginROI_2, 
        // 	nMeasurement, 
        // 	&mGrayROI_2);

        this->m_pOptimizer->AddInverseDepthEdge(
        	mPoint3d,
        	nCameraIndex_1,
        	nCameraIndex_2,
        	gOriginROI_2, 
        	nMeasurement, 
        	&mGrayROI_2);

	}
}



bool SurroundView::AddCullingEdges(int nIndex, 
					int nCameraIndex_1, int nCameraIndex_2,
					vector<cv::Mat> & gSurroundViews,
					vector<cv::Mat> & gNextSVs,
					cv::Mat & mGrayROI_1,
					cv::Mat & mGrayROI_2){
	//Check if 2 cameras are neighbourhood.
	int nCheck = nCameraIndex_1-nCameraIndex_2;
	if (nCheck*nCheck == 4){
		cout << "2 cameras are not neighbourhood!" << endl;
		return false;
	}

	//The projection is from 1 to 2.
	cv::Mat mSurroundView_1, mSurroundView_2;
	

	//Used to transfer surround-view coordinate to ground coordinate
	cv::Mat mK_G_inv = this->m_mK_G.inv();
	cv::Mat mK_G_Augment;
	cv::vconcat( mK_G_inv.rowRange(0 , 2) , cv::Mat::zeros(1 , 3 , CV_64FC1) , mK_G_Augment);
	cv::vconcat( mK_G_Augment , mK_G_inv.rowRange(2 , 3) , mK_G_Augment);

	//Generate ROI on original images.
	cv::Mat mOriginROI_1, mOriginROI_2;
	vector<int> gOriginROI_1, gOriginROI_2;
	//Get the ROI on original image and on birds-eye image.
	vector<cv::Rect> gBirdROIs = {m_iROI_FL, m_iROI_LB, m_iROI_BR, m_iROI_RF};
	//Use the right index to get the ROI.
	cv::Rect iBirdsROI;
	//Get the ROI.
	if (nCameraIndex_1 - nCameraIndex_2 == 1 ||
		nCameraIndex_1 - nCameraIndex_2 == -3 ){
		//Left is 1, right is 2.
		// index_1 = 1 , 2 , 3 , 0
		// index_2 = 0 , 1 , 2 , 3
		GetCoupleROI(nIndex, nCameraIndex_1, nCameraIndex_2,
					 mOriginROI_1, mOriginROI_2,
				 	 gOriginROI_1, gOriginROI_2);
		iBirdsROI = gBirdROIs[nCameraIndex_2];
	}else if (nCameraIndex_1 - nCameraIndex_2 == -1 ||
			  nCameraIndex_1 - nCameraIndex_2 == 3 ){
		//Left is 2, right is 1.
		// index_1 = 0 , 1 , 2 , 3
		// index_2 = 1 , 2 , 3 , 0
		GetCoupleROI(nIndex, nCameraIndex_2, nCameraIndex_1,
					 mOriginROI_2, mOriginROI_1,
				 	 gOriginROI_2, gOriginROI_1);
		iBirdsROI = gBirdROIs[nCameraIndex_1];
	}else{
		cout << "Wrong ROI pairs!" << endl;
		return false;
	}
	
	//Construct gray image.
    cv::cvtColor(mOriginROI_1,mGrayROI_1,cv::COLOR_BGR2GRAY);
    cv::cvtColor(mOriginROI_2, mGrayROI_2, cv::COLOR_BGR2GRAY);

	//The ROI on birds-eye view of this camera.
	//Used in computing the coef.
	//Get the ROI.
	cv::Mat mBirdseyeROI_1 = gSurroundViews[nCameraIndex_1](iBirdsROI);
	cv::Mat mBirdseyeROI_2 = gSurroundViews[nCameraIndex_2](iBirdsROI);

	//Convert to grayscale.
	cv::Mat mBirdseyeGray_1, mBirdseyeGray_2;
    cv::cvtColor(mBirdseyeROI_1, mBirdseyeGray_1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mBirdseyeROI_2, mBirdseyeGray_2, cv::COLOR_BGR2GRAY);
	mBirdseyeGray_1.convertTo(mBirdseyeGray_1, CV_64FC1);
	mBirdseyeGray_2.convertTo(mBirdseyeGray_2, CV_64FC1);

	//Get the coef to eliminate the affect of exposure time.
	double nCoef = cv::mean(mBirdseyeGray_2).val[0]/cv::mean(mBirdseyeGray_1).val[0];

	PixelSelection iSelection;
	// vector<cv::Point2d> gPoints = iSelection.GetPixels(mBirdseyeGray_1 , nCameraIndex_2, nCameraIndex_1);
	// vector<cv::Point2d> gPoints = iSelection.GetPixels(mBirdseyeGray_1);
	cv::Rect iEstimateROI;
	switch(nCameraIndex_2){
		case 0:{
			iEstimateROI = cv::Rect(0 , 0 , 1000 , 350);
			break;
		}
		case 1:{
			iEstimateROI = cv::Rect(0 , 0 , 350 , 1000);
			break;	
		}
		case 2:{
			iEstimateROI = cv::Rect(0 , 650 , 1000 , 350);
			break;		
		}
		case 3:{
			iEstimateROI = cv::Rect(650 , 0 , 350 , 1000);
			break;			
		}
		default: break;
	}

	cv::Mat mEstimateSV1, mEstimateSV2;
	mEstimateSV1 =  gSurroundViews[nCameraIndex_2](iEstimateROI);
	mEstimateSV2 =  gNextSVs[nCameraIndex_2](iEstimateROI);

	cv::Rect iBirdsEstimateROI( iBirdsROI.x - iEstimateROI.x,
								iBirdsROI.y - iEstimateROI.y,
								iBirdsROI.width,
								iBirdsROI.height);
	
	vector<cv::Point2d> gPoints = iSelection.GetCullingPixels(mBirdseyeGray_1,
												 			  mEstimateSV1,
												 			  mEstimateSV2,
												 			  iBirdsEstimateROI);
	vector<cv::Point2d> gPoints_2 = iSelection.GetPixels(mBirdseyeGray_1);

	if (gPoints_2.size() != gPoints.size()){
		// cv::imshow("Culling", gSurroundViews[nCameraIndex_1]);
		// cv::waitKey(0);

		cv::Mat mBirdseyeROI_1_clone1 = DrawPoints(mBirdseyeROI_1, gPoints);
		cv::Mat mBirdseyeROI_1_clone2 = DrawPoints(mBirdseyeROI_1, gPoints_2);

		// for (auto item : gPoints){
		// 	cv::circle(mBirdseyeROI_1_clone1, item, 3, cv::Scalar(100 , 0 , 0));
		// }
		// for (auto item : gPoints_2){
		// 	cv::circle(mBirdseyeROI_1_clone2, item, 3, cv::Scalar(100 , 0 , 0));
		// }
		// cv::imshow("Culling", mBirdseyeROI_1);
		stringstream ss;
		ss << nIndex << "_" << nCameraIndex_1 << "_" << nCameraIndex_2;
		string aTag = "";
		ss >> aTag;
		cv::imwrite("./culling/culling_ROI_" + aTag + "_c.jpg", mBirdseyeROI_1_clone1);
		cv::imwrite("./culling/culling_ROI_" + aTag + "_g.jpg", mBirdseyeROI_1_clone2);
		cv::waitKey(30);	
	}
	

	cout << "Edge size " << gPoints.size() << endl;
	// if (gPoints.size() < 2000){
	// 	return false;
	// }
	//Add edges in the right region.
	for (cv::Point2d iPoint2d : gPoints){
		int u = iPoint2d.x , v = iPoint2d.y;
		//Get the surround-view coordinate of the point.
		int nU = u + iBirdsROI.x;
		int nV = v + iBirdsROI.y;
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


        double nMeasurement = mBirdseyeGray_1.at<double>(v , u) * nCoef;
        //From 1 to 2

        this->m_pOptimizer->AddInverseDepthEdge(
        	mPoint3d,
        	nCameraIndex_1,
        	nCameraIndex_2,
        	gOriginROI_2, 
        	nMeasurement, 
        	&mGrayROI_2);

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

	
	// cv::Mat mGrayROI_Right,mGrayROI_Left;
	// cv::Mat mGrayROI_Right2,mGrayROI_Left2;
	// this->AddEdge(nIndex, 1, gSurroundViews, mGrayROI_Right, mGrayROI_Left);
	// this->AddEdge(nIndex, 3, gSurroundViews, mGrayROI_Right2, mGrayROI_Left2);

	cv::Mat mGrayROI_FL_1, mGrayROI_FL_2;
	cv::Mat mGrayROI_BL_1, mGrayROI_BL_2;
	cv::Mat mGrayROI_FR_1, mGrayROI_FR_2;
	cv::Mat mGrayROI_BR_1, mGrayROI_BR_2;

	//FL
	this->AddCoupleEdges(nIndex,
						 0, 1,
						 gSurroundViews, 
						 mGrayROI_FL_1, mGrayROI_FL_2);

	//BL
	this->AddCoupleEdges(nIndex,
						 2, 1,
						 gSurroundViews, 
						 mGrayROI_BL_1, mGrayROI_BL_2);

	//FR
	this->AddCoupleEdges(nIndex,
						 0, 3,
						 gSurroundViews, 
						 mGrayROI_FR_1, mGrayROI_FR_2);

	//BR
	this->AddCoupleEdges(nIndex,
						 2, 3,
						 gSurroundViews, 
						 mGrayROI_BR_1, mGrayROI_BR_2);

	
	// cout << "mGrayROI_Right size " << endl << mGrayROI_Right.rows << "*" << mGrayROI_Right.cols << endl; 

	ofstream fOutFile("../data/info.txt");
	this->m_pOptimizer->Optimize(fOutFile);
}


bool SurroundView::OptimizeWithMultiFrame(vector<int> gIndices){
	vector<cv::Mat> gGrayROI_Right, gGrayROI_Left;
	vector<cv::Mat> gGrayROI_Right2, gGrayROI_Left2;


	vector<cv::Mat> gGrayROI_FL_1, gGrayROI_FL_2;
	vector<cv::Mat> gGrayROI_BL_1, gGrayROI_BL_2;
	vector<cv::Mat> gGrayROI_FR_1, gGrayROI_FR_2;
	vector<cv::Mat> gGrayROI_BR_1, gGrayROI_BR_2;


	
	for (int i=0;i<gIndices.size();i++){
		cv::Mat mGrayROI_FL_1, mGrayROI_FL_2;
		cv::Mat mGrayROI_BL_1, mGrayROI_BL_2;
		cv::Mat mGrayROI_FR_1, mGrayROI_FR_2;
		cv::Mat mGrayROI_BR_1, mGrayROI_BR_2;

		gGrayROI_FL_1.push_back(mGrayROI_FL_1);
		gGrayROI_FL_2.push_back(mGrayROI_FL_2);

		gGrayROI_BL_1.push_back(mGrayROI_BL_1);
		gGrayROI_BL_2.push_back(mGrayROI_BL_2);

		gGrayROI_FR_1.push_back(mGrayROI_FR_1);
		gGrayROI_FR_2.push_back(mGrayROI_FR_2);

		gGrayROI_BR_1.push_back(mGrayROI_BR_1);
		gGrayROI_BR_2.push_back(mGrayROI_BR_2);
	}


	for (int i=0;i<gIndices.size();i++){
		int nIndex = gIndices[i];
		cout << "Index is " << nIndex << endl;
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


		//FL
		this->AddCoupleEdges(nIndex,
							 0, 1,
							 gSurroundViews, 
							 gGrayROI_FL_1[i],
							 gGrayROI_FL_2[i]);


		//BL
		this->AddCoupleEdges(nIndex,
							 2, 1,
							 gSurroundViews, 
							 gGrayROI_BL_1[i], 
							 gGrayROI_BL_2[i]);

		//FR
		this->AddCoupleEdges(nIndex,
							 0, 3,
							 gSurroundViews, 
							 gGrayROI_FR_1[i], 
							 gGrayROI_FR_2[i]);

		//BR
		this->AddCoupleEdges(nIndex,
							 2, 3,
							 gSurroundViews, 
							 gGrayROI_BR_1[i],
							 gGrayROI_BR_2[i]);


		
	}
	ofstream fOutFile("../data/info.txt");
	this->m_pOptimizer->Optimize(fOutFile);
}




bool SurroundView::OptimizeWithCulling(vector<int> gIndices){
	vector<cv::Mat> gGrayROI_Right, gGrayROI_Left;
	vector<cv::Mat> gGrayROI_Right2, gGrayROI_Left2;


	vector<cv::Mat> gGrayROI_FL_1, gGrayROI_FL_2;
	vector<cv::Mat> gGrayROI_BL_1, gGrayROI_BL_2;
	vector<cv::Mat> gGrayROI_FR_1, gGrayROI_FR_2;
	vector<cv::Mat> gGrayROI_BR_1, gGrayROI_BR_2;


	
	for (int i=0;i<gIndices.size();i++){
		cv::Mat mGrayROI_FL_1, mGrayROI_FL_2;
		cv::Mat mGrayROI_BL_1, mGrayROI_BL_2;
		cv::Mat mGrayROI_FR_1, mGrayROI_FR_2;
		cv::Mat mGrayROI_BR_1, mGrayROI_BR_2;

		gGrayROI_FL_1.push_back(mGrayROI_FL_1);
		gGrayROI_FL_2.push_back(mGrayROI_FL_2);

		gGrayROI_BL_1.push_back(mGrayROI_BL_1);
		gGrayROI_BL_2.push_back(mGrayROI_BL_2);

		gGrayROI_FR_1.push_back(mGrayROI_FR_1);
		gGrayROI_FR_2.push_back(mGrayROI_FR_2);

		gGrayROI_BR_1.push_back(mGrayROI_BR_1);
		gGrayROI_BR_2.push_back(mGrayROI_BR_2);
	}


	for (int i=0;i<gIndices.size();i++){
		int nIndex = gIndices[i];
		cv::Mat mSurroundView_Front = GenerateBirdsView(nIndex, 0,  1000, 1000);
		cv::Mat mSurroundView_Left = GenerateBirdsView(nIndex, 1,  1000, 1000);
		cv::Mat mSurroundView_Back = GenerateBirdsView(nIndex, 2,  1000, 1000);
		cv::Mat mSurroundView_Right = GenerateBirdsView(nIndex, 3,  1000, 1000);

		cv::Mat mNextSV_Front = GenerateBirdsView(nIndex+1, 0,  1000, 1000);
		cv::Mat mNextSV_Left = GenerateBirdsView(nIndex+1, 1,  1000, 1000);
		cv::Mat mNextSV_Back = GenerateBirdsView(nIndex+1, 2,  1000, 1000);
		cv::Mat mNextSV_Right = GenerateBirdsView(nIndex+1, 3,  1000, 1000);



		vector<cv::Mat> gSurroundViews = {
			mSurroundView_Front,
			mSurroundView_Left,
			mSurroundView_Back,
			mSurroundView_Right
		};



		vector<cv::Mat> gNextSVs = {
			mNextSV_Front,
			mNextSV_Left,
			mNextSV_Back,
			mNextSV_Right
		};


		//FL
		this->AddCullingEdges(nIndex,
							  0, 1,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_FL_1[i],
							  gGrayROI_FL_2[i]);


		//BL
		this->AddCullingEdges(nIndex,
							  2, 1,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_BL_1[i], 
							  gGrayROI_BL_2[i]);

		//FR
		this->AddCullingEdges(nIndex,
							  0, 3,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_FR_1[i], 
							  gGrayROI_FR_2[i]);

		//BR
		this->AddCullingEdges(nIndex,
							  2, 3,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_BR_1[i],
							  gGrayROI_BR_2[i]);


		
	}
	this->m_pOptimizer->Optimize();


}





vector<vector<Sophus::SE3>> SurroundView::OptimizeWithCulling(vector<int> gIndices, ofstream & fOutFile){
	vector<cv::Mat> gGrayROI_Right, gGrayROI_Left;
	vector<cv::Mat> gGrayROI_Right2, gGrayROI_Left2;


	vector<cv::Mat> gGrayROI_FL_1, gGrayROI_FL_2;
	vector<cv::Mat> gGrayROI_BL_1, gGrayROI_BL_2;
	vector<cv::Mat> gGrayROI_FR_1, gGrayROI_FR_2;
	vector<cv::Mat> gGrayROI_BR_1, gGrayROI_BR_2;


	
	for (int i=0;i<gIndices.size();i++){
		cv::Mat mGrayROI_FL_1, mGrayROI_FL_2;
		cv::Mat mGrayROI_BL_1, mGrayROI_BL_2;
		cv::Mat mGrayROI_FR_1, mGrayROI_FR_2;
		cv::Mat mGrayROI_BR_1, mGrayROI_BR_2;

		gGrayROI_FL_1.push_back(mGrayROI_FL_1);
		gGrayROI_FL_2.push_back(mGrayROI_FL_2);

		gGrayROI_BL_1.push_back(mGrayROI_BL_1);
		gGrayROI_BL_2.push_back(mGrayROI_BL_2);

		gGrayROI_FR_1.push_back(mGrayROI_FR_1);
		gGrayROI_FR_2.push_back(mGrayROI_FR_2);

		gGrayROI_BR_1.push_back(mGrayROI_BR_1);
		gGrayROI_BR_2.push_back(mGrayROI_BR_2);
	}


	for (int i=0;i<gIndices.size();i++){
		int nIndex = gIndices[i];
		cv::Mat mSurroundView_Front = GenerateBirdsView(nIndex, 0,  1000, 1000);
		cv::Mat mSurroundView_Left = GenerateBirdsView(nIndex, 1,  1000, 1000);
		cv::Mat mSurroundView_Back = GenerateBirdsView(nIndex, 2,  1000, 1000);
		cv::Mat mSurroundView_Right = GenerateBirdsView(nIndex, 3,  1000, 1000);

		cv::Mat mNextSV_Front = GenerateBirdsView(nIndex+1, 0,  1000, 1000);
		cv::Mat mNextSV_Left = GenerateBirdsView(nIndex+1, 1,  1000, 1000);
		cv::Mat mNextSV_Back = GenerateBirdsView(nIndex+1, 2,  1000, 1000);
		cv::Mat mNextSV_Right = GenerateBirdsView(nIndex+1, 3,  1000, 1000);



		vector<cv::Mat> gSurroundViews = {
			mSurroundView_Front,
			mSurroundView_Left,
			mSurroundView_Back,
			mSurroundView_Right
		};



		vector<cv::Mat> gNextSVs = {
			mNextSV_Front,
			mNextSV_Left,
			mNextSV_Back,
			mNextSV_Right
		};


		//FL
		this->AddCullingEdges(nIndex,
							  0, 1,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_FL_1[i],
							  gGrayROI_FL_2[i]);


		//BL
		this->AddCullingEdges(nIndex,
							  2, 1,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_BL_1[i], 
							  gGrayROI_BL_2[i]);

		//FR
		this->AddCullingEdges(nIndex,
							  0, 3,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_FR_1[i], 
							  gGrayROI_FR_2[i]);

		//BR
		this->AddCullingEdges(nIndex,
							  2, 3,
							  gSurroundViews, 
							  gNextSVs,
							  gGrayROI_BR_1[i],
							  gGrayROI_BR_2[i]);


		
	}
	// ofstream fOutFile("./ROECS/");

	vector<vector<Sophus::SE3>> gPoses = this->m_pOptimizer->Optimize(fOutFile);
	return gPoses;
}






