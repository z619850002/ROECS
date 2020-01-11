#ifndef CAMERA_H_
#define CAMERA_H_

//Standard.
#include <iostream>
#include <vector>
#include <string>
//Eigen.
#include <Eigen/Core>
#include <Eigen/Dense>
//Opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "sophus/se3.h"
#include "sophus/so3.h"
using namespace std;


//TODO: This may be designed by prototype mode.
class Camera{
public:
	//Constructor.
	Camera();
	Camera(	string aCameraName, Sophus::SE3 mT, 
			Eigen::Matrix3d mK, Eigen::Vector4d mD);

	bool BlurPose();

	string m_aCameraName;

	//The pose of the camera.
	Sophus::SE3 m_mT;
	//The intrinsic matrix.
	Eigen::Matrix3d m_mK;
	//The distortion coefficient.
	Eigen::Vector4d m_mD;
};




#endif