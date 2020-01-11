#ifndef INITIALIZER_h_
#define INITIALIZER_h_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.h"
#include <queue>
#include "../camera/camera.h"

class Initializer
{
public:
	Initializer();

	void InitializePose(Sophus::SE3& T_FG,Sophus::SE3& T_LG,
						Sophus::SE3& T_BG,Sophus::SE3& T_RG);
	void InitializeK(Eigen::Matrix3d& K_F, Eigen::Matrix3d& K_L,
					 Eigen::Matrix3d& K_B, Eigen::Matrix3d& K_R);
	void InitializeD(Eigen::Vector4d& D_F, Eigen::Vector4d& D_L,
					 Eigen::Vector4d& D_B, Eigen::Vector4d& D_R);	

	void InitializeCameras(	Camera * & pFrontCamera,
							Camera * & pLeftCamera,
							Camera * & pBackCamera,
							Camera * & pRightCamera);

private: 
	//The pose of the camera.
	Sophus::SE3 m_mT_FG;
	Sophus::SE3 m_mT_LG;
	Sophus::SE3 m_mT_BG;
	Sophus::SE3 m_mT_RG;

	//The intrinsic matrix.
	Eigen::Matrix3d m_mK_F;
	Eigen::Matrix3d m_mK_L;
	Eigen::Matrix3d m_mK_B;
	Eigen::Matrix3d m_mK_R;

	//The distortion coefficient.
	Eigen::Vector4d m_mD_F;
	Eigen::Vector4d m_mD_L;
	Eigen::Vector4d m_mD_B;
	Eigen::Vector4d m_mD_R;

};



#endif 