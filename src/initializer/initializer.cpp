#include "../../include/initializer/initializer.h"

using namespace std;



void initializePose(Sophus::SE3& T_FG,Sophus::SE3& T_LG,
					Sophus::SE3& T_BG,Sophus::SE3& T_RG)
{
	// Initialize T_FG
	Eigen::Matrix3d R_FG = Eigen::Matrix3d();
	R_FG<< 9.99277118e-01,  3.82390286e-04, -3.80143958e-02,
		  -2.30748265e-02, -7.88582447e-01, -6.14495953e-01,
	      -3.02124625e-02,  6.14928921e-01, -7.88003572e-01;

	Eigen::Vector3d t_FG;
	t_FG<< 6.75437418e-01,  2.50896883e+01,  3.17779305e+00;
	T_FG = Sophus::SE3(R_FG,t_FG);
	
	// Initialize T_LG
	Eigen::Matrix3d R_LG = Eigen::Matrix3d();
	R_LG<<-1.21898860e-02,  9.99924056e-01, -1.81349393e-03,
		   8.02363600e-01,  8.69913885e-03, -5.96772133e-01,
	      -5.96711036e-01, -8.72966581e-03, -8.02408707e-01;
	Eigen::Vector3d t_LG;
	t_LG<< 1.36392943e+00,  1.60942881e+01,  1.04105913e+01;
	T_LG = Sophus::SE3(R_LG,t_LG);

	
	// Initialize T_BG
	Eigen::Matrix3d R_BG = Eigen::Matrix3d();
	R_BG<<-9.99615699e-01,  1.56439861e-02, -2.28849354e-02,
		   2.59906371e-02,  8.16008735e-01, -5.77454960e-01,
	       9.64060983e-03, -5.77827838e-01, -8.16101739e-01;
	Eigen::Vector3d t_BG;
	t_BG<< 1.09266953e+00,  2.46308124e+01,  6.60957845e+00;
	T_BG = Sophus::SE3(R_BG,t_BG);

	// Initialize T_RG
	Eigen::Matrix3d R_RG = Eigen::Matrix3d();
	R_RG<< 4.57647596e-03, -9.99989102e-01,  9.22798184e-04,
		  -6.26343448e-01, -3.58584197e-03, -7.79538984e-01,
	       7.79533797e-01,  2.98955282e-03, -6.26353033e-01;
	Eigen::Vector3d t_RG;
	t_RG<<-1.66115120e-01,  1.76226207e+01, 6.08338205e+00;
	T_RG = Sophus::SE3(R_RG,t_RG);
	
	return;
}

void initializeK(Eigen::Matrix3d& K_F, Eigen::Matrix3d& K_L,
				 Eigen::Matrix3d& K_B, Eigen::Matrix3d& K_R)
{
	K_F<<422.13163849,   0.00000000, 612.82890504,
		   0.00000000, 421.10340889, 545.05656249,
		   0.00000000,   0.00000000,   1.00000000;

	K_L<<420.60079305,   0.00000000, 650.54173853,
		   0.00000000, 418.94827188, 527.27178143,
		   0.00000000,   0.00000000,   1.00000000;

	K_B<<422.61569350,   0.00000000, 632.46019501,
		   0.00000000, 421.27373079, 548.34673288,
		   0.00000000,   0.00000000,   1.00000000;

	K_R<<421.64203585,   0.00000000, 640.09362064,
		   0.00000000, 420.26647020, 529.05566315,
		   0.00000000,   0.00000000,   1.00000000;
	return;
}

void initializeD(Eigen::Vector4d& D_F, Eigen::Vector4d& D_L,
				 Eigen::Vector4d& D_B, Eigen::Vector4d& D_R)
{
	D_F<<-0.07031853,      0.00387505,     -0.00333139,     0.00056406;
	D_L<<-6.58382798e-02, -2.00728513e-03, -3.72535694e-04, 1.81851668e-06;
	D_B<<-0.06553861,     -0.00094857,     -0.00150748,     0.000325;
	D_R<<-0.07289752,      0.01254629,     -0.01300477,     0.00361266;
	return;
}






//Constructor
Initializer::Initializer(){

	//Use 1216 pose
	initializePose(	this->m_mT_FG,
					this->m_mT_LG,
					this->m_mT_BG,
					this->m_mT_RG);

	//Initialize the intrinsic
	initializeK( this->m_mK_F,
				 this->m_mK_L,
				 this->m_mK_B,
				 this->m_mK_R);

	//Initialize the distortion coefficient.
	initializeD( this->m_mD_F, 
				 this->m_mD_L,
				 this->m_mD_B,
				 this->m_mD_R);

}

void Initializer::InitializePose(Sophus::SE3& T_FG,Sophus::SE3& T_LG,
						Sophus::SE3& T_BG,Sophus::SE3& T_RG){

	T_FG = this->m_mT_FG;
	T_LG = this->m_mT_LG;
	T_BG = this->m_mT_BG;
	T_RG = this->m_mT_RG;
}



void Initializer::InitializeK(	Eigen::Matrix3d& K_F, Eigen::Matrix3d& K_L,
					 			Eigen::Matrix3d& K_B, Eigen::Matrix3d& K_R){
	K_F = this->m_mK_F;
	K_L = this->m_mK_L;
	K_B = this->m_mK_B;
	K_R = this->m_mK_R;
}


void Initializer::InitializeD(	Eigen::Vector4d& D_F, Eigen::Vector4d& D_L,
					 			Eigen::Vector4d& D_B, Eigen::Vector4d& D_R){
	D_F = this->m_mD_F;
	D_L = this->m_mD_L;
	D_B = this->m_mD_B;
	D_R = this->m_mD_R;
}



void Initializer::InitializeCameras(Camera * & pFrontCamera,
									Camera * & pLeftCamera,
									Camera * & pBackCamera,
									Camera * & pRightCamera){

	pFrontCamera = new Camera("Front", this->m_mT_FG, this->m_mK_F, this->m_mD_F);
	pLeftCamera =  new Camera("Left",this->m_mT_LG, this->m_mK_L, this->m_mD_L);
	pBackCamera =  new Camera("Back",this->m_mT_BG, this->m_mK_B, this->m_mD_B);
	pRightCamera = new Camera("Right",this->m_mT_RG, this->m_mK_R, this->m_mD_R);
}







