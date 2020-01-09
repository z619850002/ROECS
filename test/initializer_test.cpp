//Test initializer
#include <iostream>
#include "../include/initializer/initializer.h"

using namespace std;


int main(){
	Initializer iInitializer;
	
	Sophus::SE3 mT_FG;
	Sophus::SE3 mT_LG;
	Sophus::SE3 mT_BG;
	Sophus::SE3 mT_RG;

	//Test pose
	iInitializer.InitializePose(mT_FG, mT_LG , mT_BG , mT_RG);

	cout << "T_FG: " << endl << mT_FG << endl;
	cout << "T_LG: " << endl << mT_LG << endl;
	cout << "T_BG: " << endl << mT_BG << endl;
	cout << "T_RG: " << endl << mT_RG << endl;

	//Test intrinsics.
	Eigen::Matrix3d mK_F;
	Eigen::Matrix3d mK_L;
	Eigen::Matrix3d mK_B;
	Eigen::Matrix3d mK_R;

	iInitializer.InitializeK(mK_F, mK_L, mK_B, mK_R);

	cout << "K_F: " << endl << mK_F << endl;
	cout << "K_L: " << endl << mK_L << endl;
	cout << "K_B: " << endl << mK_B << endl;
	cout << "K_R: " << endl << mK_R << endl;


	//Test distortion.

	Eigen::Vector4d mD_F;
	Eigen::Vector4d mD_L;
	Eigen::Vector4d mD_B;
	Eigen::Vector4d mD_R;

	iInitializer.InitializeD(mD_F, mD_L, mD_B, mD_R);

	cout << "D_F: " << endl << mD_F << endl;
	cout << "D_L: " << endl << mD_L << endl;
	cout << "D_B: " << endl << mD_B << endl;
	cout << "D_R: " << endl << mD_R << endl;

	return 0;
}