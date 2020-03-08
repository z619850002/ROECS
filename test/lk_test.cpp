
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>



#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"
#include <vector>

#include <iostream>


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <g2o/types/slam2d/vertex_se2.h>

using namespace std;








void ExtractFeatures(cv::Mat mImage1, cv::Mat mImage2, vector<cv::DMatch> & gMatches,
						vector<cv::KeyPoint> & gKeyPoints1, vector<cv::KeyPoint> & gKeyPoints2){
    
    cv::Mat mDescriptors1, mDescriptors2;
    cv::Ptr<cv::FeatureDetector> pDetector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> pDescriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> pMatcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

    //Detect the corner
    pDetector->detect(mImage1, gKeyPoints1);
    pDetector->detect(mImage2, gKeyPoints2);

    //Compute the descriptor
    pDescriptor->compute(mImage1, gKeyPoints1, mDescriptors1);
    pDescriptor->compute(mImage2, gKeyPoints2, mDescriptors2);


    pMatcher->match(mDescriptors1, mDescriptors2, gMatches);


    // 仅供娱乐的写法
    double nMinDist = min_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    double nMaxDist = max_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    


    vector< cv::DMatch > gGoodMatches;
    for ( int i = 0; i < mDescriptors1.rows; i++ )
    {
        if ( gMatches[i].distance <= max ( 3*nMinDist, 30.0 ) )
        {
            gGoodMatches.push_back ( gMatches[i] );
        }
    }

    gMatches.clear();
    for (auto item : gGoodMatches){
    	gMatches.push_back(item);
    }

    cv::Mat mGoodImage;
    cv::drawMatches ( mImage1, gKeyPoints1, mImage2, gKeyPoints2, gMatches, mGoodImage );
    cv::imshow("const cv::String &winname", mGoodImage);
    cv::waitKey(0);


}

 


int main(){
	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


	pLeftCamera->BlurPose();
	pRightCamera->BlurPose();


	FrameLoader iLoader("/home/kyrie/Documents/Data/1216" , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	cout << "Load pairs" << endl;
	vector<SVPair> gPairs = iLoader.LoadFramePairs(vector<int>{0 , 1 , 2 , 3 , 4});

	SurroundView iSurround(pFrontCamera, pLeftCamera, pBackCamera, pRightCamera);

	cout << "Bind images" << endl;
	iSurround.BindImagePairs(gPairs);

	cout << "Init K_G" << endl;
	iSurround.InitK_G(1000, 1000, 0.1, 0.1);

	cout << "Finish init K_G" << endl;
	// cv::Mat mSurroundView = iSurround.GenerateSurroundView(3, 1000, 1000);
	// cv::imshow("const cv::String &winname", mSurroundView);
	// cv::waitKey(0);



	// mSurroundView = iSurround.GenerateSurroundView(4, 1000, 1000);
	// cv::imshow("const cv::String &winname", mSurroundView);
	// cv::waitKey(0);


	cv::Mat mFrontBirdseye = iSurround.GenerateBirdsView(3, 0, 1000, 1000);
	cv::imshow("const cv::String &winname", mFrontBirdseye);
	cv::waitKey(0);


	cv::Mat mFrontBirdseye2 = iSurround.GenerateBirdsView(4, 0, 1000, 1000);
	cv::imshow("const cv::String &winname", mFrontBirdseye2);
	cv::waitKey(0);



	cv::Mat mLeftBirdseye = iSurround.GenerateBirdsView(3, 1, 1000, 1000);
	cv::imshow("const cv::String &winname", mLeftBirdseye);
	cv::waitKey(0);


	cv::Mat mLeftBirdseye2 = iSurround.GenerateBirdsView(4, 1, 1000, 1000);
	cv::imshow("const cv::String &winname", mLeftBirdseye2);
	cv::waitKey(0);


	cv::Mat mFrontBirdseyeROI_1 = mFrontBirdseye(cv::Rect(100 , 0 , 800 , 400));
	cv::Mat mFrontBirdseyeROI_2 = mFrontBirdseye2(cv::Rect(100 , 0 , 800 , 400));

	cv::Mat mLeftBirdseyeROI_1 = mLeftBirdseye(iSurround.m_iROI_FL);
	cv::Mat mLeftBirdseyeROI_2 = mLeftBirdseye2(iSurround.m_iROI_FL);

	// cv::imshow("const cv::String &winname", mFrontBirdseyeROI_1);
	// cv::waitKey(0);
	// cv::imshow("const cv::String &winname", mFrontBirdseyeROI_2);
	// cv::waitKey(0);

	// cv::imshow("const cv::String &winname", mLeftBirdseyeROI_1);
	// cv::waitKey(0);
	// cv::imshow("const cv::String &winname", mLeftBirdseyeROI_2);
	// cv::waitKey(0);

	cv::Mat mFrontGray_1, mFrontGray_2;
	cv::Mat mLeftGray_1, mLeftGray_2;

	cv::cvtColor(mFrontBirdseyeROI_1, mFrontGray_1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mFrontBirdseyeROI_2, mFrontGray_2, cv::COLOR_BGR2GRAY);
	mFrontGray_1.convertTo(mFrontGray_1, CV_64FC1);
	mFrontGray_2.convertTo(mFrontGray_2, CV_64FC1);



	cv::cvtColor(mLeftBirdseyeROI_1, mLeftGray_1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mLeftBirdseyeROI_2, mLeftGray_2, cv::COLOR_BGR2GRAY);
	mLeftGray_1.convertTo(mLeftGray_1, CV_64FC1);
	mLeftGray_2.convertTo(mLeftGray_2, CV_64FC1);



	vector<cv::DMatch> gMatches;
	vector<cv::KeyPoint> gKeyPoints1, gKeyPoints2;
	ExtractFeatures(mFrontBirdseyeROI_1, mFrontBirdseyeROI_2, gMatches,
					gKeyPoints1, gKeyPoints2);


	// cv::DMatch iMatch;
	// iMatch.tra
	vector<Eigen::Vector2d> gPoints1, gPoints2;
	for (cv::DMatch iMatch : gMatches){
		cv::Point2f iPoint1 = gKeyPoints1[iMatch.queryIdx].pt;
		cv::Point2f iPoint2 = gKeyPoints2[iMatch.trainIdx].pt;
		

		double nDistance = (iPoint1.x - iPoint2.x) * (iPoint1.x - iPoint2.x);
		nDistance += (iPoint1.y - iPoint2.y) * (iPoint1.y - iPoint2.y);
		cout << "Distance is " << nDistance << endl;
		if (nDistance >= 500 || nDistance <=50){
			continue;
		}
		gPoints1.push_back(
			Eigen::Vector2d(iPoint1.x + 100, iPoint1.y)
			);
		gPoints2.push_back(
			Eigen::Vector2d(iPoint2.x + 100, iPoint2.y)
			);
	}


	cv::Mat mMergedImage = mFrontBirdseyeROI_1;
	cv::vconcat(mMergedImage, mFrontBirdseyeROI_2, mMergedImage);
	cout << "size " << endl << mMergedImage.size() << endl;
	for (int i=0;i<gPoints1.size();i++){
		cv::Point2f iPoint2 = cv::Point2f(gPoints2[i](0), gPoints2[i](1));
		iPoint2.y = iPoint2.y + 400;
		iPoint2.x = iPoint2.x - 100;
		cv::Point2f iPoint1 = cv::Point2f(gPoints1[i](0), gPoints1[i](1));
		iPoint1.x = iPoint1.x - 100;
		cv::line(mMergedImage, iPoint1, iPoint2, cv::Scalar(100 , 0 , 0));
	}

	cv::imshow("const cv::String &winname", mMergedImage);
	cv::waitKey(0);






	// PixelSelection iSelection;
	// vector<cv::Point2d> gPointsFront = iSelection.GetPixels(mFrontGray_1);
	// gPointsFront.clear();


	// vector<cv::KeyPoint> gKeyPoints;
 //    cv::Ptr<cv::FastFeatureDetector> pDetector = cv::FastFeatureDetector::create();
 //    pDetector->detect( mFrontBirdseyeROI_1, gKeyPoints );
 //     for ( auto kp:gKeyPoints )
 //     {
 //        gPointsFront.push_back( kp.pt );
 //     }


	// for (auto iPoint : gPointsFront){
	// 	cv::circle(mFrontBirdseyeROI_1, iPoint, 2, cv::Scalar(0 , 240 , 0) , 1);
	// }

	// cv::imshow("const cv::String &winname", mFrontBirdseyeROI_1);
	// cv::waitKey(0);
	

	// vector<unsigned char> gStatus;
 //    vector<float> gError;
 //    vector<cv::Point2f> gNextPoints;
 //    vector<cv::Point2f> gCurrentPoints;

 //    for (auto item : gPointsFront){
 //    	gCurrentPoints.push_back(cv::Point2f(item.x, item.y));
 //    }
     




	// cv::calcOpticalFlowPyrLK( mFrontBirdseyeROI_1, mFrontBirdseyeROI_2, gCurrentPoints, gNextPoints, gStatus, gError);


	// for (int i=0;i<gNextPoints.size();i++){
	// 	cv::Point2f iPoint = gNextPoints[i];
	// 	cv::Point2f iPreviousPoint = gCurrentPoints[i];
	// 	cv::circle(mFrontBirdseyeROI_2, iPoint, 5, cv::Scalar(0 , 240 , 0) , 1);
	// 	cv::line(mFrontBirdseyeROI_2, iPreviousPoint, iPoint, cv::Scalar(0 , 240 , 0) , 1);
	// }

	// // for (auto iPoint : gNextPoints){
	// // 	cv::circle(mFrontBirdseyeROI_2, iPoint, 2, cv::Scalar(0 , 240 , 0) , 1);
	// // 	cv::line(mFrontBirdseyeROI_2, , Point pt2, const Scalar &color)
	// // }

	// cv::imshow("const cv::String &winname", mFrontBirdseyeROI_2);
	// cv::waitKey(0);
	


	//Optimization process.


	typedef g2o::BlockSolverX DirectBlock;
    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ());
    // std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
    std::unique_ptr<DirectBlock> pSolverPtr (new DirectBlock ( std::move(pLinearSolver) ));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* pSolver = new g2o::OptimizationAlgorithmLevenberg ( std::move(pSolverPtr) ); // L-M
	//TODO: These configuration should be adjusted in the future.    
    g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm ( pSolver );
    iOptimizer.setVerbose( true );


    g2o::VertexSE2 * pVertexSE2 = new g2o::VertexSE2();
    pVertexSE2->setEstimate(Eigen::Vector3d(0 , 0 , 0));
    pVertexSE2->setId(1000);
    iOptimizer.addVertex(pVertexSE2);



    for (int i=0;i<gPoints1.size();i++){
    	Eigen::Vector2d iPoint1 = gPoints1[i];
    	Eigen::Vector2d iPoint2 = gPoints2[i];
    	g2o::VertexPointXY * pPointXY = new g2o::VertexPointXY();
    	pPointXY->setEstimate(iPoint1);
    	pPointXY->setId(i);
    	iOptimizer.addVertex(pPointXY);

    	g2o::EdgeSE2PointXY * pEdge = new g2o::EdgeSE2PointXY();
    	pEdge->setMeasurementData(iPoint2.data());
    	pEdge->setInformation(Eigen::Matrix2d::Identity());
    	pEdge->setVertex(0, pVertexSE2);
    	pEdge->setVertex(1, pPointXY);
    	g2o::RobustKernel * pKernel = new g2o::RobustKernelCauchy;
        pKernel->setDelta(23.3);
        pEdge->setRobustKernel(pKernel);
    	iOptimizer.addEdge(pEdge);
    }

    cout << "Edge number" << endl << iOptimizer.edges().size() << endl;


	iOptimizer.initializeOptimization();
    iOptimizer.optimize(100);


    // g2o::VertexPointXY
    Eigen::Isometry2d mRotation = pVertexSE2->estimate().inverse().toIsometry();
    Eigen::Matrix3d mPose = mRotation.matrix();
    cout << "Pose is " << endl;
    cout << mPose << endl;
	
    cv::Mat mPose_CV;
    cv::eigen2cv(mPose, mPose_CV);
    cv::Mat mGeneratedImage = mFrontBirdseyeROI_1.clone();
    for (int u=0;u<mGeneratedImage.cols;u++){
    	for (int v=0;v<mGeneratedImage.rows;v++){
    		cv::Mat mPos = (cv::Mat_<double>(3 , 1) << u , v , 1);
    		mPos = mPose_CV.inv() * mPos;
    		int nNewU = mPos.at<double>(0 , 0);
    		int nNewV = mPos.at<double>(1 , 0);
    		if (nNewU <= 1 || nNewV <=1 || 
    			nNewU >= mGeneratedImage.cols-1 ||
    			nNewV >= mGeneratedImage.rows-1){
    			mGeneratedImage.at<cv::Vec3b>(v , u) = cv::Vec3b(0 , 0 , 0);
    		}else{
    			mGeneratedImage.at<cv::Vec3b>(v , u) = mFrontBirdseyeROI_1.at<cv::Vec3b>(nNewV, nNewU);
    		}
    	}
    }




	PixelSelection iSelection;

	cv::Mat mCull = mFrontBirdseyeROI_2.clone();
	cv::Mat mUnCull = mFrontBirdseyeROI_2.clone();

	vector<cv::Point2d> gPointsFront = iSelection.GetPixels(mFrontGray_2);
	for (auto iPoint : gPointsFront){
		int u = iPoint.x;
		int v = iPoint.y;


		cv::Vec3b mColorPointA = mFrontBirdseyeROI_2.at<cv::Vec3b>(v , u);
		cv::Vec3b mColorPointB = mGeneratedImage.at<cv::Vec3b>(v , u);

		if (mColorPointB[0] == 0){
			mColorPointB[0] = 1;
		}

		if (mColorPointB[1] == 0){
			mColorPointB[1] = 1;
		}

		if (mColorPointB[2] == 0){
			mColorPointB[2] = 1;
		}

		float nScale1 = (float)mColorPointA[0]/(float)mColorPointB[0];
		float nScale2 = (float)mColorPointA[1]/(float)mColorPointB[1];
		float nScale3 = (float)mColorPointA[2]/(float)mColorPointB[2];
		float nAverageScale = (nScale1 + nScale2 + nScale3)/3;
		float nSigma = sqrt((nScale1 - nAverageScale) * (nScale1 - nAverageScale) + 
							(nScale2 - nAverageScale) * (nScale2 - nAverageScale) + 
							(nScale3 - nAverageScale) * (nScale3 - nAverageScale));


		
		if (nSigma < 0.03){
			cv::circle(mCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
			cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
			cout << "nSigma is " << nSigma << endl;
		}else{
			cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
		}
	}

	cv::imshow("const cv::String &winname2", mFrontBirdseyeROI_2);
	cv::imshow("const cv::String &winname", mUnCull);
	cv::imwrite("uncull.jpg", mUnCull);
	cv::waitKey(0);
	cv::imshow("const cv::String &winname", mCull);
	cv::imwrite("cull.jpg", mCull);
	cv::waitKey(0);
	
	// cv::imshow("const cv::String &winname", mGeneratedImage);
	// cv::waitKey(0);



	return 0;
}









