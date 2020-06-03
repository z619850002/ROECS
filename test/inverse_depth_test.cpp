#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <vector>
#include <iostream>
#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"

using namespace std;


Eigen::Vector3d Project_Pixel(Camera * pCamera , Eigen::Vector4d mPoint_Ground){
	Eigen::Vector4d mPoint_Camera = pCamera->m_mT.matrix() * mPoint_Ground;

	Eigen::Vector3d mPoint_Pixel = pCamera->m_mK * 
			Eigen::Vector3d(
				mPoint_Camera[0]/mPoint_Camera[2],
				mPoint_Camera[1]/mPoint_Camera[2],
				1
		);
	return Eigen::Vector3d(mPoint_Pixel[0] - 1100 , mPoint_Pixel[1] - 300 , 1);
	//Left Cx = -1100 , Cy = -300	
}


Eigen::Vector3d Project_Camera(Camera * pCamera , Eigen::Vector4d mPoint_Ground){
	Eigen::Vector4d mPoint_Camera = pCamera->m_mT.matrix() * mPoint_Ground;

	return
			Eigen::Vector3d(
				mPoint_Camera[0],
				mPoint_Camera[1],
				mPoint_Camera[2]
		);

}




float getPixelValue(float x , float y , cv::Mat * m_pImage){
    uchar* data = & m_pImage->data[ int ( y ) * m_pImage->step + int ( x ) ];
    float xx = x - floor ( x );
    float yy = y - floor ( y );
    float result = float (
               ( 1-xx ) * ( 1-yy ) * data[0] +
               xx* ( 1-yy ) * data[1] +
               ( 1-xx ) *yy*data[ m_pImage->step ] +
               xx*yy*data[m_pImage->step+1]
           );
    return result;
}



int main2(){

	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


	cv::Mat mK_G = cv::Mat::zeros(3,3,CV_64FC1);
	mK_G.at<double>(0,0) = 1/0.1;
	mK_G.at<double>(1,1) = -1/0.1;
	mK_G.at<double>(0,2) = 1000/2;
	mK_G.at<double>(1,2) = 1000/2;
	mK_G.at<double>(2,2) =   1.0;


	cv::Mat mPoint_Ground = mK_G.inv() *  (cv::Mat_<double>(3 , 1) << 100 , 100 , 1);
	cout << "Point Ground is " << endl << mPoint_Ground << endl;

	Eigen::Vector4d mPoint_Ground_Eigen = Eigen::Vector4d(
			mPoint_Ground.at<double>(0 , 0),
			mPoint_Ground.at<double>(1 , 0),
			mPoint_Ground.at<double>(2 , 0),
			1
		);



	cv::Mat mImage = cv::Mat::zeros(200,200,CV_8UC1);

	for (int u=0;u<200;u++){
		for (int v=0;v<200;v++){
			double nDistance = sqrt((u-100)*(u-100) + (v-100) * (v-100));
			mImage.at<uchar>(v , u) = nDistance*1.5;
		}
	}



	Eigen::Vector3d mPixel = Project_Pixel(pLeftCamera, mPoint_Ground_Eigen);

	double nValue = mImage.at<uchar>((int)mPixel[0] , (int)mPixel[1]);

	cout << "nValue is " << nValue << endl;


	Eigen::Vector3d mPoint_Camera_Front = Project_Camera(pFrontCamera, mPoint_Ground_Eigen);
	cout << "mPoint_Camera_Front is " << endl << mPoint_Camera_Front << endl;

	double nInverseDepth = 1/mPoint_Camera_Front[2];
	cout << "InitialInverseDepth is " << nInverseDepth << endl;

	

	double nNormalized_U = mPoint_Camera_Front[0] * nInverseDepth;
	double nNormalized_V = mPoint_Camera_Front[1] * nInverseDepth;

	nInverseDepth += 0.003;

	//Firstly create the inverse depth vertex.
	InverseDepthVertex * pInverseDepth = new InverseDepthVertex();
	pInverseDepth->setId(0);
	pInverseDepth->BindParameters(nNormalized_U,
								  nNormalized_V,
								  pFrontCamera->m_mT);

    pInverseDepth->setEstimate(nInverseDepth);

	// pInverseDepth->setFixed(true);


    //Construct the edge.
    double nFx, nFy, nCx, nCy;
    //Get the intrinsics.
    nFx = pLeftCamera->m_mK(0 , 0);
    nFy = pLeftCamera->m_mK(1 , 1);
    nCx = pLeftCamera->m_mK(0 , 2) - 1100;
    nCy = pLeftCamera->m_mK(1 , 2) - 300;


    InverseDepthEdge* pEdge = new InverseDepthEdge();
	pEdge->BindParameters(nFx,
        nFy,
        nCx,
        nCy,
        &mImage,0);

	g2o::VertexSE3Expmap * pPoseLeft = new g2o::VertexSE3Expmap();
    pPoseLeft->setEstimate ( g2o::SE3Quat ( 	pLeftCamera->m_mT.rotation_matrix() ,
    								   			pLeftCamera->m_mT.translation() ));




    //Left ID is 1.
    pPoseLeft->setId ( 20 );

    //Add the edge.
	pEdge->setVertex ( 0, pPoseLeft );
	pEdge->setVertex ( 1, pInverseDepth);
    pEdge->setMeasurement (nValue);
    pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
    pEdge->setId ( 1000);


    typedef g2o::BlockSolverX DirectBlock;
    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ());
    // std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
    std::unique_ptr<DirectBlock> pSolverPtr (new DirectBlock ( std::move(pLinearSolver) ));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* pSolver = new g2o::OptimizationAlgorithmLevenberg ( std::move(pSolverPtr) ); // L-M
    // g2o::OptimizationAlgorithmDogleg* pSolver = new g2o::OptimizationAlgorithmDogleg ( std::move(pSolverPtr) ); // L-M
	//TODO: These configuration should be adjusted in the future.    
	g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm ( pSolver );
    iOptimizer.setVerbose( true );


    iOptimizer.addVertex(pPoseLeft);

    iOptimizer.addVertex(pInverseDepth);
    iOptimizer.addEdge(pEdge);


    pPoseLeft->setFixed(true);
    // pInverseDepth->setFixed(true);


	cout << "Edge number" << endl << iOptimizer.edges().size() << endl;


	iOptimizer.initializeOptimization();
    iOptimizer.optimize ( 20 );


    double nLastInverseDepth = pInverseDepth->estimate();
    cout << "LastInverseDepth is " << nLastInverseDepth << endl;


    Eigen::Vector4d mPoint_First(mPoint_Camera_Front[0]/mPoint_Camera_Front[2]/nInverseDepth , mPoint_Camera_Front[1]/mPoint_Camera_Front[2]/nInverseDepth , 1/nInverseDepth , 1);


    Eigen::Vector4d mPoint_Last(mPoint_Camera_Front[0]/mPoint_Camera_Front[2]/nLastInverseDepth , mPoint_Camera_Front[1]/mPoint_Camera_Front[2]/nLastInverseDepth , 1/nLastInverseDepth , 1);



    // cout << "First position: " << endl << 
    Eigen::Vector4d mGround_First = pFrontCamera->m_mT.matrix().inverse() * mPoint_First;
    Eigen::Vector4d mGround_Last =  pFrontCamera->m_mT.matrix().inverse() * mPoint_Last;


    Eigen::Vector3d mFirstRes = Project_Pixel(pLeftCamera, mGround_First);
    Eigen::Vector3d mLastRes = Project_Pixel(pLeftCamera, mGround_Last);
    cout << "mFirstRes is " << endl << mFirstRes << endl;
    cout << "mLastRes is " << endl << mLastRes << endl;
    cout << "mOrigin is " << endl << mPixel << endl;


    Eigen::Vector3d mTest = pPoseLeft->estimate().map(pInverseDepth->Get3DPoint());

    //Map the point on the image.
	float nUU = mTest[0] * nFx / mTest[2] + nCx;
	float nVV = mTest[1] * nFy / mTest[2] + nCy;

	cout << "uu is " << nUU << endl << "vv is " << nVV << endl;

	cout << "First Value 1 " << getPixelValue(mFirstRes[0] , mFirstRes[1] , &mImage) << endl;
	cout << "Last Value 1 " << getPixelValue(mLastRes[0] , mLastRes[1] , &mImage) << endl;
	cout << "Origin Value 1 " << getPixelValue(mPixel[0] , mPixel[1] , &mImage) << endl;



	// cv::circle(mImage, cv::Point2f(nUU , nVV), 3, cv::Scalar(100 , 0 , 0));
	// cv::circle(mImage, cv::Point2f(mPixel[0] , mPixel[1]), 3, cv::Scalar(100 , 0 , 0));
	// cv::circle(mImage, cv::Point2f(mFirstRes[0] , mFirstRes[1]), 3, cv::Scalar(100 , 0 , 0));
	// cv::imshow("const cv::String &winname", mImage);
	// cv::waitKey(0);
	return 0;
}


int main(){

	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


	cv::Mat mK_G = cv::Mat::zeros(3,3,CV_64FC1);
	mK_G.at<double>(0,0) = 1/0.1;
	mK_G.at<double>(1,1) = -1/0.1;
	mK_G.at<double>(0,2) = 1000/2;
	mK_G.at<double>(1,2) = 1000/2;
	mK_G.at<double>(2,2) =   1.0;


	cv::Mat mPoint_Ground = mK_G.inv() *  (cv::Mat_<double>(3 , 1) << 100 , 100 , 1);
	cout << "Point Ground is " << endl << mPoint_Ground << endl;

	Eigen::Vector4d mPoint_Ground_Eigen = Eigen::Vector4d(
			mPoint_Ground.at<double>(0 , 0),
			mPoint_Ground.at<double>(1 , 0),
			mPoint_Ground.at<double>(2 , 0),
			1
		);

	Eigen::Vector4d mPoint_Ground_Eigen_Disturb = Eigen::Vector4d(
			mPoint_Ground.at<double>(0 , 0) + 3,
			mPoint_Ground.at<double>(1 , 0) +2 ,
			mPoint_Ground.at<double>(2 , 0) + 1,
			1
		);



	cv::Mat mImage = cv::Mat::zeros(200,200,CV_8UC1);

	for (int u=0;u<200;u++){
		for (int v=0;v<200;v++){
			double nDistance = sqrt((u-100)*(u-100) + (v-100) * (v-100));
			mImage.at<uchar>(v , u) = nDistance * 1.5;
		}
	}



	Eigen::Vector3d mPixel = Project_Pixel(pLeftCamera, mPoint_Ground_Eigen);

	cout << "mPixel is " << endl << mPixel <<  endl;
	double nValue = (double)mImage.at<uchar>((int)mPixel[0] , (int)mPixel[1]);

	cout << "nValue is " << nValue << endl;



	//Firstly create the inverse depth vertex.
	// InverseDepthVertex * pInverseDepth = new InverseDepthVertex();
	// pInverseDepth->setId(0);
	// pInverseDepth->BindParameters(nNormalized_U,
	// 							  nNormalized_V,
	// 							  pFrontCamera->m_mT);

 //    pInverseDepth->setEstimate(nInverseDepth);

    g2o::VertexSBAPointXYZ * pPoint = new g2o::VertexSBAPointXYZ();
    pPoint->setEstimate(Eigen::Vector3d(mPoint_Ground_Eigen_Disturb[0],
    									mPoint_Ground_Eigen_Disturb[1],
    									mPoint_Ground_Eigen_Disturb[2]));

    pPoint->setId(1);

    //Construct the edge.
    double nFx, nFy, nCx, nCy;
    //Get the intrinsics.
    nFx = pLeftCamera->m_mK(0 , 0);
    nFy = pLeftCamera->m_mK(1 , 1);
    nCx = pLeftCamera->m_mK(0 , 2) - 1100;
    nCy = pLeftCamera->m_mK(1 , 2) - 300;



	g2o::VertexSE3Expmap * pPoseLeft = new g2o::VertexSE3Expmap();
    pPoseLeft->setEstimate ( g2o::SE3Quat ( 	pLeftCamera->m_mT.rotation_matrix() ,
    								   			pLeftCamera->m_mT.translation() ));


    

    //Left ID is 1.
    pPoseLeft->setId ( 20 );


    DirectBinaryEdge* pEdge = new DirectBinaryEdge();
	pEdge->BindParameters(nFx,
        nFy,
        nCx,
        nCy,
        &mImage);


    //Add the edge.
	pEdge->setVertex ( 0, pPoseLeft );
	pEdge->setVertex ( 1, pPoint);
    pEdge->setMeasurement (nValue);
    pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
    pEdge->setId ( 1000);


    typedef g2o::BlockSolverX DirectBlock;
    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ());
    // std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
    std::unique_ptr<DirectBlock> pSolverPtr (new DirectBlock ( std::move(pLinearSolver) ));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* pSolver = new g2o::OptimizationAlgorithmLevenberg ( std::move(pSolverPtr) ); // L-M
    // g2o::OptimizationAlgorithmDogleg* pSolver = new g2o::OptimizationAlgorithmDogleg ( std::move(pSolverPtr) ); // L-M
	//TODO: These configuration should be adjusted in the future.    
	g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm ( pSolver );
    iOptimizer.setVerbose( true );


    iOptimizer.addVertex(pPoseLeft);

    iOptimizer.addVertex(pPoint);
    iOptimizer.addEdge(pEdge);


    pPoseLeft->setFixed(true);
    // pInverseDepth->setFixed(true);


	cout << "Edge number" << endl << iOptimizer.edges().size() << endl;


	iOptimizer.initializeOptimization();
    iOptimizer.optimize ( 20 );



    // cout << "First position: " << endl << 
    Eigen::Vector4d mGround_First = mPoint_Ground_Eigen_Disturb;
    Eigen::Vector4d mGround_Last =  Eigen::Vector4d(pPoint->estimate()[0],
    												pPoint->estimate()[1],
    												pPoint->estimate()[2],
    												1);


    Eigen::Vector3d mFirstRes = Project_Pixel(pLeftCamera, mGround_First);
    Eigen::Vector3d mLastRes = Project_Pixel(pLeftCamera, mGround_Last);
    cout << "mFirstRes is " << endl << mFirstRes << endl;
    cout << "mLastRes is " << endl << mLastRes << endl;
    cout << "mOrigin is " << endl << mPixel << endl;


    Eigen::Vector3d mTest = pPoseLeft->estimate().map(Eigen::Vector3d(mPoint_Ground_Eigen[0] , mPoint_Ground_Eigen[1] , mPoint_Ground_Eigen[2]));

    //Map the point on the image.
	float nUU = mTest[0] * nFx / mTest[2] + nCx;
	float nVV = mTest[1] * nFy / mTest[2] + nCy;

	cout << "uu is " << nUU << endl << "vv is " << nVV << endl;

	cout << "First Value 1 " << getPixelValue(mFirstRes[0] , mFirstRes[1] , &mImage) << endl;
	cout << "Last Value 1 " << getPixelValue(mLastRes[0] , mLastRes[1] , &mImage) << endl;
	cout << "Origin Value 1 " << getPixelValue(mPixel[0] , mPixel[1] , &mImage) << endl;



	// cv::circle(mImage, cv::Point2f(nUU , nVV), 3, cv::Scalar(100 , 0 , 0));
	// cv::circle(mImage, cv::Point2f(mLastRes[0] , mLastRes[1]), 3, cv::Scalar(100 , 0 , 0));
	// cv::circle(mImage, cv::Point2f(mFirstRes[0] , mFirstRes[1]), 3, cv::Scalar(100 , 0 , 0));
	// cv::imshow("const cv::String &winname", mImage);
	// cv::waitKey(0);
	return 0;
}