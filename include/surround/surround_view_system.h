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
using namespace std;

class SurroundView
{
public:
	//Default constructor.
	SurroundView();
	SurroundView(	Camera * pFrontCamera, Camera * pLeftCamera, 
					Camera * pBackCamera, Camera * pRightCamera);

	//Generate one surround view image
	cv::Mat GenerateSurroundView(int nIndex, int nRows, int nCols);
	//Init the K_G matrix.
	bool InitK_G(int nRows, int nCols, float nDx, float nDy);

	//Bind the image pairs, equal to a setter.
	bool BindImagePairs(vector<SVPair> gDistortedPairs);


	//Cameras in the surround-view system.
	Camera * m_pFrontCamera;
	Camera * m_pLeftCamera;
	Camera * m_pBackCamera;
	Camera * m_pRightCamera;

	//The converter matrix from ground coordinate to surround-view coordinate.
	cv::Mat m_mK_G;

	vector<SVPair> m_gDistortedPairs;
	
};


#endif