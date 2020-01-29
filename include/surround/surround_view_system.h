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

	bool GetUndistortedROI(int nIndex, int nCameraIndex, cv::Mat & mROI_Left, cv::Mat & mROI_Right,
							vector<int> & gROI_Left , vector<int> & gROI_Right);




	bool OptimizePoseWithOneFrame(int nIndex){
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


		for (int u=0;u<m_iROI_FL.width;u++){
			for (int v=0;v<m_iROI_FL.height;v++){
				//Get the surround-view coordinate of the point.
				int nU = u + m_iROI_FL.x;
				int nV = v + m_iROI_FL.y;
				cv::Mat mp_surround = (cv::Mat_<double>(3 , 1) << nU, nV, 1);
				//Convert surround-view coordinate to ground coordinate.
				cv::Mat mP_G = mK_G_Augment * mp_surround;
				
			}		
		}

	}


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
	
};


#endif