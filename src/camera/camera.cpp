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
