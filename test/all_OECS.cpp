#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"
#include <sstream>
#include <fstream>
#include <ostream>
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


	FrameLoader iLoader("/home/kyrie/Documents/Data/204-604" , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	cout << "Load pairs" << endl;
	vector<int> gIndices = {};
	for (int i=0;i<100;i++){
		gIndices.push_back(i);
	}
	vector<SVPair> gPairs = iLoader.LoadFramePairs(gIndices);



	for (int nImageIndex = 1; nImageIndex < 80; nImageIndex++){
		

		SurroundView iSurround(pFrontCamera, pLeftCamera, pBackCamera, pRightCamera);

		cout << "Bind images" << endl;
		iSurround.BindImagePairs(gPairs);

		cout << "Init K_G" << endl;
		iSurround.InitK_G(1000, 1000, 0.1, 0.1);

		cout << "Finish init K_G" << endl;

		stringstream ss;
		string aImageIndex = "";
		ss << nImageIndex;
		ss >> aImageIndex;
		while(aImageIndex.size() < 4){
			aImageIndex = "0" + aImageIndex;
		}

		ofstream iOutFile("./OECS/" + aImageIndex +  "_pose.txt");


		cv::Mat mSurroundView = iSurround.GenerateSurroundView(nImageIndex, 1000, 1000);
		cv::imwrite("./OECS/" + aImageIndex +  "_before.jpg" , mSurroundView);


		cv::imshow("surround-view", mSurroundView);
		cv::waitKey(30);
		

		// iSurround.OptimizePoseWithOneFrame(nImageIndex);

		vector<int> gOptimizeIndices = {};
		for (int i=0;i<=20;i+=2){
			gOptimizeIndices.push_back(nImageIndex + i);
		}
		// iSurround.OptimizeWithMultiFrame(gOptimizeIndices);

		// iSurround.OptimizeWithCulling(gOptimizeIndices, iOutFile);
		iSurround.OptimizePoseWithOneFrame(nImageIndex);


		mSurroundView = iSurround.GenerateSurroundView(nImageIndex, 1000, 1000);
		cv::imwrite("./OECS/" + aImageIndex +  "_after.jpg" , mSurroundView);

		cv::imshow("surround-view", mSurroundView);
		cv::waitKey(30);

		// iOutFile << "Iterations: " << aImageIndex << endl;
		// iOutFile << "Left:" << endl;
		// iOutFile << pLeftCamera->m_mT.matrix() << endl;
		// iOutFile << "Right:" << endl;
		// iOutFile << pRightCamera->m_mT.matrix() << endl;

		iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


		pLeftCamera->BlurPose();
		pRightCamera->BlurPose();
	
	}

	

	return 0;
}