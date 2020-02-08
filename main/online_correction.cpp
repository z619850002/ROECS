#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"

using namespace std;




int main(int argc , char * argv[]){

	if (argc<3){
		cerr << "Please input the directory name and the image index" << endl;
		return 0;
	}

	string aDirectoryName = argv[1];
	cout << "Directory Name is " << aDirectoryName << endl;


	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);

	int nSampleIndex = std::stoi(argv[2]);
	cout << "Index is " << nSampleIndex << endl;


	pLeftCamera->BlurPose();
	pRightCamera->BlurPose();


	FrameLoader iLoader(aDirectoryName , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	cout << "Load pairs" << endl;
	vector<SVPair> gPairs = iLoader.LoadAll();

	SurroundView iSurround(pFrontCamera, pLeftCamera, pBackCamera, pRightCamera);

	cout << "Bind images" << endl;
	iSurround.BindImagePairs(gPairs);

	cout << "Init K_G" << endl;
	iSurround.InitK_G(1000, 1000, 0.1, 0.1);

	cout << "Finish init K_G" << endl;
	cv::Mat mSurroundView = iSurround.GenerateSurroundView(nSampleIndex, 1000, 1000);
	cv::imshow("const cv::String &winname", mSurroundView);
	cv::waitKey(0);

	iSurround.OptimizePoseWithOneFrame(nSampleIndex);
	

	mSurroundView = iSurround.GenerateSurroundView(nSampleIndex, 1000, 1000);
	cv::imshow("const cv::String &winname", mSurroundView);
	cv::waitKey(0);


	return 0;
}