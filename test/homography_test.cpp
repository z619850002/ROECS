#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"
#include <sstream>
#include <fstream>
#include <ostream>
using namespace std;



#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


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

    cv::Ptr<cv::FeatureDetector> pDetector = cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::DescriptorExtractor> pDescriptor = cv::xfeatures2d::SIFT::create();
    // cv::Ptr<cv::DescriptorMatcher> pMatcher  = cv::DescriptorMatcher::create ( "BruteForce" );

    cv::Ptr<cv::BFMatcher> pMatcher = cv::BFMatcher::create(cv::NORM_L2 , true);
    

    //Detect the corner
    pDetector->detect(mImage1, gKeyPoints1);
    pDetector->detect(mImage2, gKeyPoints2);
    cout << "Finish detect" << endl;

    //Compute the descriptor
    pDescriptor->compute(mImage1, gKeyPoints1, mDescriptors1);
    pDescriptor->compute(mImage2, gKeyPoints2, mDescriptors2);
    cout  << "Finish compute" << endl;

    cout << "Descriptor size " << mDescriptors1.size() << endl;

    cout << "Descriptor size " << mDescriptors2.size() << endl;

    pMatcher->match(mDescriptors1, mDescriptors2, gMatches);
    cout << "Finish match" << endl;
    cout << "gMatches size" << gMatches.size() << endl;
    // 仅供娱乐的写法
    double nMinDist = min_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    double nMaxDist = max_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    sort(gMatches.begin(),gMatches.end() , 
    	[](cv::DMatch iMatch1 , cv::DMatch iMatch2){ return iMatch1.distance < iMatch2.distance;});
    
    cout << "MinDist is " << nMinDist << endl;
    cout << "MaxDist is " << nMaxDist << endl;

    // vector< cv::DMatch > gGoodMatches;
    // for ( int i = 0; i < mDescriptors1.rows; i++ )
    // {
    //     // if ( gMatches[i].distance <= 3*nMinDist )
    //     // {
    //         gGoodMatches.push_back ( gMatches[i] );
    //     // }
    // }

    // gMatches.clear();
    // for (auto item : gGoodMatches){
    // 	gMatches.push_back(item);
    // }

    cv::Mat mGoodImage;
    // cv::drawMatches ( mImage1, gKeyPoints1, mImage2, gKeyPoints2, gMatches, mGoodImage );
    // cv::imshow("const cv::String &winname", mGoodImage);
    // cv::waitKey(0);


}


void GeneratePointsPair(cv::Mat mBirdseyeView_1, cv::Mat mBirdseyeView_2,
						int nStartX, int nStartY,
						vector<Eigen::Vector2d> & gPoints1, vector<Eigen::Vector2d> & gPoints2){
	vector<cv::DMatch> gMatches;
	vector<cv::KeyPoint> gKeyPoints1, gKeyPoints2;
	//Extract features and then match them.
	ExtractFeatures(mBirdseyeView_1, mBirdseyeView_2, gMatches,
					gKeyPoints1, gKeyPoints2);

	cv::Mat mConcat = mBirdseyeView_1.clone();
	cv::vconcat(mConcat, mBirdseyeView_2 , mConcat);
	

	for (cv::DMatch iMatch : gMatches){
		cv::Point2f iPoint1 = gKeyPoints1[iMatch.queryIdx].pt;
		cv::Point2f iPoint2 = gKeyPoints2[iMatch.trainIdx].pt;
		
		//Use a threshold to eliminate inliers.
		double nDistance = (iPoint1.x - iPoint2.x) * (iPoint1.x - iPoint2.x);
		nDistance += (iPoint1.y - iPoint2.y) * (iPoint1.y - iPoint2.y);
		cout << "Distance is " << nDistance << endl;
		if (nDistance >= 5000){
			continue;
		}
		cv::line(mConcat, iPoint1, cv::Point2f(iPoint2.x , iPoint2.y + 400),
				 cv::Scalar(100 , 0 , 0));
		//Relative to left-top
		// gPoints1.push_back(
		// 	Eigen::Vector2d(iPoint1.x + nStartX, iPoint1.y)
		// 	);
		// gPoints2.push_back(
		// 	Eigen::Vector2d(iPoint2.x + nStartY, iPoint2.y)
		// 	);
		gPoints1.push_back(
			Eigen::Vector2d(iPoint1.x, iPoint1.y)
			);
		gPoints2.push_back(
			Eigen::Vector2d(iPoint2.x, iPoint2.y)
			);
	}

	// cv::imshow("const cv::String &winname", mConcat);
	// cv::waitKey(0);

	
}

 


void OptimizeForPose(	vector<Eigen::Vector2d> & gPoints1,
						vector<Eigen::Vector2d> & gPoints2,
						cv::Mat & mPose_CV){


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



	iOptimizer.initializeOptimization();
    iOptimizer.optimize(100);


    // g2o::VertexPointXY
    Eigen::Isometry2d mIsometry = pVertexSE2->estimate().inverse().toIsometry();
    
    Eigen::Matrix3d mPose = mIsometry.matrix();
    cv::eigen2cv(mPose, mPose_CV);
}


void OptimizeHomographyPose(	vector<Eigen::Vector2d> & gPoints1,
								vector<Eigen::Vector2d> & gPoints2,
								cv::Mat & mPose_CV){
	vector<cv::Point2f> gPoints1_CV, gPoints2_CV;
	for (int i=0;i<gPoints2.size();i++){
		Eigen::Vector2d iPoint1 = gPoints1[i];
		Eigen::Vector2d iPoint2 = gPoints2[i];
		gPoints1_CV.push_back(cv::Point2f(iPoint1(0) , iPoint1(1)));
		gPoints2_CV.push_back(cv::Point2f(iPoint2(0) , iPoint2(1)));

		cout << "Point1 " << gPoints1[i] << endl;
		cout << "Point2 " << gPoints2[i] << endl;
	}

	cv::Mat mHomography = cv::findHomography(gPoints1_CV, gPoints2_CV , cv::RANSAC ,5);
	cout << "mHomography is " << endl << mHomography << endl;
	mPose_CV = mHomography.clone();

}


void CopyImage(cv::Mat & mFromImage,
			   cv::Mat & mGeneratedImage,
			   cv::Mat & mPose_CV,
			   int nStartX, int nStartY){
    mGeneratedImage = mFromImage.clone();
    for (int u=0;u<mGeneratedImage.cols;u++){
    	for (int v=0;v<mGeneratedImage.rows;v++){
    		
    		cv::Mat mPos = (cv::Mat_<double>(3 , 1) << u , v , 1);
    		
    		mPos = mPose_CV *  mPos;
    		 int nNewU = mPos.at<double>(0 , 0);
    		int nNewV =  mPos.at<double>(1 , 0);
    		if (nNewU <= 1 || nNewV <=1 || 
    			nNewU >= mGeneratedImage.cols-1 ||
    			nNewV >= mGeneratedImage.rows-1){
    			mGeneratedImage.at<cv::Vec3b>(v , u) = cv::Vec3b(0 , 0 , 0);
    		}else{
    			mGeneratedImage.at<cv::Vec3b>(v , u) = mFromImage.at<cv::Vec3b>(nNewV, nNewU);
    		}
    	}
    }

    // cv::imshow("mFromImage", mFromImage);
    // cv::imwrite("from.jpg", mFromImage);
    // cv::waitKey(0);
    // cv::imshow("mFromImage", mGeneratedImage);
    // cv::imwrite("generated.jpg", mGeneratedImage);
    // cv::waitKey(0);

}

double CalculateSigma(cv::Mat mImage1 , cv::Mat mGeneratedImage, cv::Point2f iPoint){
		int u = iPoint.x;
		int v = iPoint.y;

		int nCol = mImage1.cols;
		int nRow = mImage1.rows;

		int nSize = 0;
		double nTotalSigma = 0.0;
		for (int i = -6;i<=6;i+=3){
			for (int j=-6;j<=6;j+=3){
				if (u + i <= 0 || v + j <=0 || u+i >= nCol || v+j >= nRow){
					continue;
				}
				cv::Vec3b mColorPointA = mImage1.at<cv::Vec3b>(v+i , u+i);
				cv::Vec3b mColorPointB = mGeneratedImage.at<cv::Vec3b>(v+i , u+i);

				if (mColorPointB[0] == 0 && 
					mColorPointB[1] == 0 && 
					mColorPointB[2] == 0){
					continue;
				}

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
				nTotalSigma += nSigma;
				nSize++;
			}
		}

		if (nSize == 0){
			return 0;
		}

		return nTotalSigma / nSize;

		
}



void Culling(cv::Mat & mImage1, cv::Mat & mImage2, int nStartX, int nStartY){

	//Get gray scale image.


	cv::Mat mGray1, mGray2;
	

	cv::cvtColor(mImage1, mGray1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mImage2, mGray2, cv::COLOR_BGR2GRAY);
	mGray1.convertTo(mGray1, CV_64FC1);
	mGray2.convertTo(mGray2, CV_64FC1);




	vector<Eigen::Vector2d> gPoints1, gPoints2;
	GeneratePointsPair(	mImage1, mImage2,
	 					nStartX, nStartY,
	 					gPoints1, gPoints2);






	//Optimization process.
	cv::Mat mPose_CV;
	// OptimizeForPose(gPoints1, gPoints2, mPose_CV);
	OptimizeHomographyPose(gPoints1, gPoints2, mPose_CV);
	cout << "Pose is " << endl << mPose_CV << endl;
	
	cv::Mat mGeneratedImage;
	CopyImage(mImage2, mGeneratedImage, mPose_CV, nStartX, nStartY);



	PixelSelection iSelection;

	cv::Mat mCull = mImage1.clone();
	cv::Mat mUnCull = mImage1.clone();

	vector<cv::Point2d> gPointsFront = iSelection.GetPixels(mGray1);

	vector<float> gSigma;
	for (auto iPoint : gPointsFront){
		double nSigma = CalculateSigma(mImage1, mGeneratedImage, iPoint);
		// int u = iPoint.x;
		// int v = iPoint.y;



		// cv::Vec3b mColorPointA = mImage1.at<cv::Vec3b>(v , u);
		// cv::Vec3b mColorPointB = mGeneratedImage.at<cv::Vec3b>(v , u);

		// if (mColorPointB[0] == 0){
		// 	mColorPointB[0] = 1;
		// }

		// if (mColorPointB[1] == 0){
		// 	mColorPointB[1] = 1;
		// }

		// if (mColorPointB[2] == 0){
		// 	mColorPointB[2] = 1;
		// }

		// float nScale1 = (float)mColorPointA[0]/(float)mColorPointB[0];
		// float nScale2 = (float)mColorPointA[1]/(float)mColorPointB[1];
		// float nScale3 = (float)mColorPointA[2]/(float)mColorPointB[2];
		// float nAverageScale = (nScale1 + nScale2 + nScale3)/3;
		// float nSigma = sqrt((nScale1 - nAverageScale) * (nScale1 - nAverageScale) + 
		// 					(nScale2 - nAverageScale) * (nScale2 - nAverageScale) + 
		// 					(nScale3 - nAverageScale) * (nScale3 - nAverageScale));


		// cout << "nSigma is " << nSigma << endl;
		gSigma.push_back(nSigma);
		// if (nSigma < 0.03){
		// 	cv::circle(mCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
		// 	cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
			
		// }else{
		// 	cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
		// }
	}


	double nTotal, nMeanSigma;
	//Get the mean gSigma.
	double nSig = 0.0;

	int nNum = 0;
	for (auto item : gSigma){
		if (item < 1){
			nTotal += item;
			nNum++;
		}
	}
	nMeanSigma = nTotal /(float)nNum;

	for (auto item : gSigma){
		if (item < 1){
			nSig += ((item-nMeanSigma) * (item-nMeanSigma))/(float)nNum;
		}
	}
	nSig = sqrt(nSig);
	cout << "Mean sigma is " << nTotal /(float)nNum << endl;
	cout << "sig is " << nSig << endl;

	for (int i=0;i<gSigma.size();i++){
		float nSigma = gSigma[i];
		cv::Point2d iPoint = gPointsFront[i];
		if (nSigma < nMeanSigma){
			cv::circle(mCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
			cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
			
		}else{
			cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
		}
	}

	// cv::imshow("const cv::String &winname2", mImage1);
	// // cv::imwrite("from.jpg" , mImage1);
	// cv::waitKey(0);
	// cv::imshow("const cv::String &winname2", mGeneratedImage);
	// // cv::imwrite("generated.jpg" , mGeneratedImage);
	// cv::waitKey(0);

	cv::imshow("const cv::String &winname", mUnCull);
	// cv::imwrite("uncull.jpg", mUnCull);
	cv::waitKey(0);
	cv::imshow("const cv::String &winname", mCull);
	// cv::imwrite("cull.jpg", mCull);
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


	FrameLoader iLoader("/home/kyrie/Documents/Data/all" , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	cout << "Load pairs" << endl;
	vector<int> gIndices = {};
	for (int i=0;i<100;i++){
		gIndices.push_back(i);
	}
	vector<SVPair> gPairs = iLoader.LoadFramePairs(gIndices);


	for (int nImageIndex = 35; nImageIndex < 100; nImageIndex++){
		SurroundView iSurround(pFrontCamera, pLeftCamera, pBackCamera, pRightCamera);

		cout << "Bind images" << endl;
		iSurround.BindImagePairs(gPairs);

		cout << "Init K_G" << endl;
		iSurround.InitK_G(1000, 1000, 0.1, 0.1);

		cout << "Finish init K_G" << endl;

		


		cv::Mat mFrontBirdseye = iSurround.GenerateBirdsView(nImageIndex, 0, 1000, 1000);
		cv::Mat mFrontBirdseye2 = iSurround.GenerateBirdsView(nImageIndex+1, 0, 1000, 1000);
		cv::Mat mLeftBirdseye = iSurround.GenerateBirdsView(nImageIndex, 1, 1000, 1000);
		cv::Mat mLeftBirdseye2 = iSurround.GenerateBirdsView(nImageIndex+1, 1, 1000, 1000);
		cv::Mat mRightBirdseye = iSurround.GenerateBirdsView(nImageIndex, 3, 1000, 1000);
		cv::Mat mRightBirdseye2 = iSurround.GenerateBirdsView(nImageIndex+1, 3, 1000, 1000);
		cv::Mat mBackBirdseye = iSurround.GenerateBirdsView(nImageIndex, 2, 1000, 1000);
		cv::Mat mBackBirdseye2 = iSurround.GenerateBirdsView(nImageIndex+1, 2, 1000, 1000);
		
		cv::Mat mFrontBirdseyeROI_1 = mFrontBirdseye(cv::Rect(0 , 0 , 1000 , 300));
		cv::Mat mFrontBirdseyeROI_2 = mFrontBirdseye2(cv::Rect(0 , 0 , 1000 , 300));

		cv::Mat mLeftBirdseyeROI_1 = mLeftBirdseye(cv::Rect(0 , 0 , 300 , 1000));
		cv::Mat mLeftBirdseyeROI_2 = mLeftBirdseye2(cv::Rect(0 , 0 , 300 , 1000));


		cv::Mat mRightBirdseyeROI_1 = mRightBirdseye(cv::Rect(700 , 0 , 300 , 1000));
		cv::Mat mRightBirdseyeROI_2 = mRightBirdseye2(cv::Rect(700 , 0 , 300 , 1000));


		cv::Mat mBackBirdseyeROI_1 = mBackBirdseye(cv::Rect(0 , 700 , 1000 , 300));
		cv::Mat mBackBirdseyeROI_2 = mBackBirdseye2(cv::Rect(0 , 700 , 1000 , 300));




		Culling(mFrontBirdseyeROI_1, mFrontBirdseyeROI_2, 0 , 0);

		Culling(mLeftBirdseyeROI_1, mLeftBirdseyeROI_2, 0 , 0);

		Culling(mBackBirdseyeROI_1, mBackBirdseyeROI_2, 0 , 700);

		Culling(mRightBirdseyeROI_1, mRightBirdseyeROI_2, 700 , 0);




		iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


		pLeftCamera->BlurPose();
		pRightCamera->BlurPose();
	
	}

	

	return 0;
}