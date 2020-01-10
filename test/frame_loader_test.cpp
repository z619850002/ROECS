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

	FrameLoader iLoader("/home/kyrie/Documents/Data/1216" , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	
	vector<SVPair> gPairs = iLoader.LoadFramePairs(vector<int>{0 , 1 , 2});

	cv::imshow("first front", gPairs[0].m_pFrontFrame->m_mFisheyeImage);
	cv::waitKey(0);

	cv::imshow("first left", gPairs[0].m_pLeftFrame->m_mFisheyeImage);
	cv::waitKey(0);
	cv::imshow("first back", gPairs[0].m_pBackFrame->m_mFisheyeImage);
	cv::waitKey(0);
	cv::imshow("first right", gPairs[0].m_pRightFrame->m_mFisheyeImage);
	cv::waitKey(0);


	return 0;
}