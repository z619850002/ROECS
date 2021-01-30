#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"

using namespace std;




int main(int argc , char * argv[]){

	if (argc<2){
		cerr << "Please input the directory name" << endl;
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

	iSurround.StartPipeline();
	// vector<int> gIndices = {5 , 10 , 15 , 20 , 25};
	// iSurround.OptimizeWithCulling(gIndices);

	// cv::Mat mSurroundView = iSurround.GenerateSurroundView(1, 1000, 1000);
	// cv::imshow("const string &winname", mSurroundView);
	// cv::waitKey(0);
		
	


	return 0;
}