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

using namespace std;


//TODO: This may be designed by prototype mode.
class Camera{
public:
	//Constructor.
	Camera();
	
	//Setter.
	void SetName(string aCameraName);
	void SetR(cv::Mat mMat);
	void SetT(cv::Mat mVec);
	void SetK(cv::Mat mK);
	void SetD(cv::Mat mD);
	//Getter.
	string GetName();
	cv::Mat GetIsometry();
	cv::Mat GetRotation();
	cv::Mat GetTranslation();
	cv::Mat GetK();
	cv::Mat GetD();
private:
	string m_aCameraName;
	//The pose of the camera.
	cv::Mat m_mR;
	cv::Mat m_mt;
	//Intrinsics.
	cv::Mat m_mK;
	//Distortion.
	cv::Mat m_mD;
};




#endif