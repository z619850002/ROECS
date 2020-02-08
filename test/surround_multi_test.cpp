#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"

using namespace std;




int main(){

	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


	pLeftCamera->BlurPose();
	pRightCamera->BlurPose();


	FrameLoader iLoader("/home/kyrie/Documents/Data/1216" , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	cout << "Load pairs" << endl;
	vector<SVPair> gPairs = iLoader.LoadFramePairs(vector<int>{0 , 1 , 2 , 3});

	SurroundView iSurround(pFrontCamera, pLeftCamera, pBackCamera, pRightCamera);

	cout << "Bind images" << endl;
	iSurround.BindImagePairs(gPairs);

	cout << "Init K_G" << endl;
	iSurround.InitK_G(1000, 1000, 0.1, 0.1);

	cout << "Finish init K_G" << endl;
	cv::Mat mSurroundView = iSurround.GenerateSurroundView(0, 1000, 1000);
	cv::imshow("const cv::String &winname", mSurroundView);
	cv::waitKey(0);

	// iSurround.OptimizePoseWithOneFrame(2);
	vector<int> gIndices = {0,1,2,3};
	iSurround.OptimizeWithMultiFrame(gIndices);

	mSurroundView = iSurround.GenerateSurroundView(0, 1000, 1000);
	cv::imshow("const cv::String &winname", mSurroundView);
	cv::waitKey(0);


	return 0;
}