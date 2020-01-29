#include "../../include/surround/surround_view_system.h"
#include "utils.h"
#include "../../include/optimizer/direct_unary_edge.h"

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
	cout << "ROI left is " << endl;
	for (auto item : gROI_Left){
		cout << item << endl;
	}


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



bool SurroundView::OptimizePoseWithOneFrame(int nIndex){
	//Generate birds-eye view image.
	//Useless.
	cv::Mat mSurroundView_Front = GenerateBirdsView(nIndex, 0,  1000, 1000);
	cv::Mat mSurroundView_Left = GenerateBirdsView(nIndex, 1,  1000, 1000);
	cv::Mat mSurroundView_Back = GenerateBirdsView(nIndex, 2,  1000, 1000);
	cv::Mat mSurroundView_Right = GenerateBirdsView(nIndex, 3,  1000, 1000);

	//Now use the point on the ground to construct the optimization structure.
	//Firstly use the left view to test.
	cv::Mat mROI_FL, mROI_LB;
	vector<int> gROI_FL, gROI_LB;
	//Get the ROI.
	GetUndistortedROI(nIndex, 1, mROI_FL, mROI_LB, gROI_FL, gROI_LB);

	//Used to transfer surround-view coordinate to ground coordinate
	cv::Mat mK_G_inv = this->m_mK_G.inv();
	cv::Mat mK_G_Augment;
	cv::vconcat( mK_G_inv.rowRange(0 , 2) , cv::Mat::zeros(1 , 3 , CV_64FC1) , mK_G_Augment);
	cv::vconcat( mK_G_Augment , mK_G_inv.rowRange(2 , 3) , mK_G_Augment);

	//Construct gray image.
    cv::Mat mGrayROI_FL , mGrayROI_LB;
    cv::cvtColor(mROI_FL,mGrayROI_FL,cv::COLOR_BGR2GRAY);
    cv::cvtColor(mROI_LB, mGrayROI_LB, cv::COLOR_BGR2GRAY);

	//Construct the optimizer.
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    std::unique_ptr<DirectBlock::LinearSolverType> linearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
    std::unique_ptr<DirectBlock> solver_ptr (new DirectBlock ( std::move(linearSolver) ));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr) ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( true );

    // Vertex of pose.
    cout << "Add pose vertex" << endl;
    g2o::VertexSE3Expmap* pPose = new g2o::VertexSE3Expmap();
    pPose->setEstimate ( g2o::SE3Quat ( m_pLeftCamera->m_mT.rotation_matrix() ,
    								   m_pLeftCamera->m_mT.translation() ));

    cout << "Rotation is " << endl << m_pLeftCamera->m_mT.rotation_matrix() << endl;
    cout << "Translation is " << endl << m_pLeftCamera->m_mT.translation() << endl;

    pPose->setId ( 0 );
    optimizer.addVertex ( pPose );


    cout << "Add edges." << endl;
	//Front-left points.
	//The depth is fixed now.
    cv::Mat mFrontROI_FL = mSurroundView_Front(m_iROI_FL);
    cv::cvtColor(mFrontROI_FL, mFrontROI_FL, cv::COLOR_BGR2GRAY);
	cv::Mat mLeftROI_FL = mSurroundView_Left(m_iROI_FL);
	cv::cvtColor(mLeftROI_FL, mLeftROI_FL, cv::COLOR_BGR2GRAY);
	mFrontROI_FL.convertTo(mFrontROI_FL, CV_64FC1);
	mLeftROI_FL.convertTo(mLeftROI_FL, CV_64FC1);
	//Get the coef to eliminate the affect of exposure time.
	double nCoef = cv::mean(mLeftROI_FL).val[0]/cv::mean(mFrontROI_FL).val[0];

	int nEdgeID = 1;

	for (int u=0;u<m_iROI_FL.width;u++){
		for (int v=0;v<m_iROI_FL.height;v++){
			//Get the surround-view coordinate of the point.
			int nU = u + m_iROI_FL.x;
			int nV = v + m_iROI_FL.y;
			cv::Mat mp_surround = (cv::Mat_<double>(3 , 1) << nU, nV, 1);
			//Convert surround-view coordinate to ground coordinate.
			cv::Mat mP_G = mK_G_Augment * mp_surround;


            Eigen::Vector3d mPoint3d(mP_G.at<double>(0 , 0),
            						 mP_G.at<double>(1 , 0),
            						 mP_G.at<double>(2 , 0));


            double nFx, nFy, nCx, nCy;
            nFx = m_pLeftCamera->m_mK(0 , 0);
            nFy = m_pLeftCamera->m_mK(1 , 1);
            nCx = m_pLeftCamera->m_mK(0 , 2) - gROI_FL[0];
            nCy = m_pLeftCamera->m_mK(1 , 2) - gROI_FL[1];

			DirectUnaryEdge* pEdge = new DirectUnaryEdge (
	            mPoint3d,
	            nFx,
	            nFy,
	            nCx,
	            nCy,
	            &mGrayROI_FL
	        );

	        pEdge->setVertex ( 0, pPose );
	        //TODO: The measurement needs to be obtained.
	        double nMeasurement = mFrontROI_FL.at<double>(v , u) * nCoef;
	        pEdge->setMeasurement (nMeasurement);
	        pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
	        pEdge->setId ( nEdgeID++ );
	        optimizer.addEdge ( pEdge );
		}		
	}


	cv::Mat mBackROI_LB = mSurroundView_Back(m_iROI_LB);
    cv::cvtColor(mBackROI_LB, mBackROI_LB, cv::COLOR_BGR2GRAY);
	cv::Mat mLeftROI_LB = mSurroundView_Left(m_iROI_LB);
	cv::cvtColor(mLeftROI_LB, mLeftROI_LB, cv::COLOR_BGR2GRAY);
	mLeftROI_LB.convertTo(mLeftROI_LB, CV_64FC1);
	mBackROI_LB.convertTo(mBackROI_LB, CV_64FC1);
	//Get the coef to eliminate the affect of exposure time.
	double nCoef2 = cv::mean(mLeftROI_LB).val[0]/cv::mean(mBackROI_LB).val[0];

	for (int u=0;u<m_iROI_LB.width;u++){
		for (int v=0;v<m_iROI_LB.height;v++){
			//Get the surround-view coordinate of the point.
			int nU = u + m_iROI_LB.x;
			int nV = v + m_iROI_LB.y;
			cv::Mat mp_surround = (cv::Mat_<double>(3 , 1) << nU, nV, 1);
			//Convert surround-view coordinate to ground coordinate.
			cv::Mat mP_G = mK_G_Augment * mp_surround;


            Eigen::Vector3d mPoint3d(mP_G.at<double>(0 , 0),
            						 mP_G.at<double>(1 , 0),
            						 mP_G.at<double>(2 , 0));


            double nFx, nFy, nCx, nCy;
            nFx = m_pLeftCamera->m_mK(0 , 0);
            nFy = m_pLeftCamera->m_mK(1 , 1);
            nCx = m_pLeftCamera->m_mK(0 , 2) - gROI_LB[0];
            nCy = m_pLeftCamera->m_mK(1 , 2) - gROI_LB[1];

			DirectUnaryEdge* pEdge = new DirectUnaryEdge (
	            mPoint3d,
	            nFx,
	            nFy,
	            nCx,
	            nCy,
	            &mGrayROI_LB
	        );


	        pEdge->setVertex ( 0, pPose );
	        //TODO: The measurement needs to be obtained.
	        double nMeasurement = mBackROI_LB.at<double>(v , u) * nCoef2;

	        pEdge->setMeasurement (nMeasurement);
	        pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
	        pEdge->setId ( nEdgeID++ );
	        optimizer.addEdge ( pEdge );
		}		
	}

	//Begin to optimize.
	cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    cout << "Before optimization, pose is " << endl << m_pLeftCamera->m_mT.matrix() << endl;
    cout << "After optimiztion, pose is " << endl << pPose->estimate() << endl;

    Eigen::Isometry3d mTcw = pPose->estimate();
    m_pLeftCamera->m_mT = Sophus::SE3(mTcw.rotation() , mTcw.translation());

}