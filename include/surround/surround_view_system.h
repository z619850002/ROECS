#ifndef SURROUND_VIEW_SYSTEM_H_
#define SURROUND_VIEW_SYSTEM_H_


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "sophus/se3.h"
#include "sophus/so3.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>




#include <iostream>
#include <vector>
#include <string>

#include "../camera/camera.h"
#include "../frame/frame_pair.h"
#include "../optimizer/direct_unary_edge.h"
#include "../../include/optimizer/surround_optimizer.h"
#include "../../include/selection/pixel_selection.h"
using namespace std;

class SurroundView
{
public:
	//Default constructor.
	SurroundView();
	SurroundView(	Camera * pFrontCamera, Camera * pLeftCamera, 
					Camera * pBackCamera, Camera * pRightCamera);

	//Generate one surround view image.
	cv::Mat GenerateSurroundView(int nIndex, int nRows, int nCols);
	//Generate one birds-eye view image.
	cv::Mat GenerateBirdsView(int nIndex, int nCamera, int nRows, int nCols);
	//Init the K_G matrix.
	bool InitK_G(int nRows, int nCols, float nDx, float nDy);

	//Bind the image pairs, equal to a setter.
	bool BindImagePairs(vector<SVPair> gDistortedPairs);



	//Get the undistort ROI of all projection to one camera, totally 2 couples.
	bool GetUndistortedROI(int nIndex, int nCameraIndex, cv::Mat & mROI_Left, cv::Mat & mROI_Right,
							vector<int> & gROI_Left , vector<int> & gROI_Right);
	//Get the undistort ROI of one couple. Like FL, BR.
	bool GetCoupleROI(int nIndex, int nCameraIndex_Left, int nCameraIndex_Right,
					  cv::Mat & mROI_Left, cv::Mat & mROI_Right,
					  vector<int> & gROI_Left, vector<int> & gROI_Right);



	bool GetBirdseyeROI(int nIndex, int nCameraIndex, cv::Mat & mROI_Left, cv::Mat & mROI_Right);


	//Add all edges projecting on one camera.
	bool AddEdge(int nIndex, int nCameraIndex,
							vector<cv::Mat> & gSurroundViews,
							cv::Mat & mGrayROI_Right,
							cv::Mat & mGrayROI_Left);

	//Add a couple of edges.
	bool AddCoupleEdges(int nIndex, 
					int nCameraIndex_1, int nCameraIndex_2,
					vector<cv::Mat> & gSurroundViews,
					cv::Mat & mGrayROI_1,
					cv::Mat & mGrayROI_2);


	//Use one pair of frames to optimize the pose.
	bool OptimizePoseWithOneFrame(int nIndex);
	//Use multiple frames to optimize the pose.
	bool OptimizeWithMultiFrame(vector<int> gIndices);


	//Cameras in the surround-view system.
	Camera * m_pFrontCamera;
	Camera * m_pLeftCamera;
	Camera * m_pBackCamera;
	Camera * m_pRightCamera;

	//The converter matrix from ground coordinate to surround-view coordinate.
	cv::Mat m_mK_G;

	//ROI of 4 overlapping region.
	cv::Rect m_iROI_FL;
	cv::Rect m_iROI_LB;
	cv::Rect m_iROI_BR;
	cv::Rect m_iROI_RF;

	vector<SVPair> m_gDistortedPairs;

	SurroundOptimizer * m_pOptimizer;
	
};


#endif