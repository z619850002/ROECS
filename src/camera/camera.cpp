#include "../../include/camera/camera.h"

using namespace std;

//Constructor.
Camera::Camera(){
}



Camera::Camera(	string aCameraName, Sophus::SE3 mT, 
				Eigen::Matrix3d mK, Eigen::Vector4d mD)
	:m_mT(mT), m_mK(mK), m_mD(mD), m_aCameraName(aCameraName)
{

}


bool Camera::BlurPose(){
	//Add a disturbance to the pose of the camera.
	Eigen::Matrix<double,6,1>  V6;
	V6<<0.01, -0.01, 0.01, -0.01, 0.01, -0.01;
	this->m_mT = Sophus::SE3::exp(this->m_mT.log()+V6*2);

	return true;
}
