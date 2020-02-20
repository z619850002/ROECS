#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"

using namespace std;



// Matcher::Matcher(float nMatchRatio)
// 	: m_nMatchRatio(nMatchRatio) , m_pMatcher(cv::Ptr<cv::BFMatcher>(new cv::BFMatcher(cv::NORM_HAMMING)))
// {

// }


// void Matcher::Match(cv::Mat mDescriptors1 , cv::Mat mDescriptors2 , vector<cv::DMatch> & gMatches){


// 	this->m_pMatcher->match(mDescriptors1, mDescriptors2, gMatches);
// 	//Get the best matches
	
// 	//Use the lamda expression to get the min distance.
// 	float nMinDistance = min_element(	gMatches.begin(), gMatches.end(),
// 										[] (const cv::DMatch &m1 , const cv::DMatch &m2)
// 			{
// 				return m1.distance < m2.distance;	
// 			}
// 		)->distance;

// 	float nMatchRatio = m_nMatchRatio;

// 	// <trainIdx, minDistance>
// 	map<int , float> mMinDistanceCompress;

// 	for (auto iMatch : gMatches){
// 		if (mMinDistanceCompress.find(iMatch.trainIdx) == mMinDistanceCompress.end()){
// 			mMinDistanceCompress[iMatch.trainIdx] = iMatch.distance;
// 		}else if (mMinDistanceCompress[iMatch.trainIdx] >iMatch.distance){
// 			mMinDistanceCompress[iMatch.trainIdx] = iMatch.distance;
// 		}
// 	}



// 	//Filter.
// 	//Erase all invalid matches.
// 	gMatches.erase(std::remove_if(gMatches.begin() , gMatches.end(),
// 						[nMinDistance , nMatchRatio, mMinDistanceCompress] (cv::DMatch & m)
// 					{
// 						float threshold = max<float> (nMinDistance * nMatchRatio , 30.0);
// 						return !((m.distance < threshold) && 
// 								  (m.distance <= mMinDistanceCompress.at(m.trainIdx)));
// 					}) , gMatches.end());

// 	//If multiple feature points are matched with the same point, use the best one.


// }






int main(){

	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


	// pLeftCamera->BlurPose();
	// pRightCamera->BlurPose();


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
	cv::Mat mSurroundView = iSurround.GenerateSurroundView(2, 1000, 1000);
	cv::imshow("const cv::String &winname", mSurroundView);
	cv::waitKey(0);

	// // iSurround.OptimizePoseWithOneFrame(2);

	// mSurroundView = iSurround.GenerateSurroundView(2, 1000, 1000);
	// cv::imshow("const cv::String &winname", mSurroundView);
	// cv::waitKey(0);


	cv::Mat mROIFL_F, mROIFL_L;
	iSurround.GetBirdseyeROI(2, 0, mROIFL_L, mROIFL_F);

	cv::imshow("F", mROIFL_F);
	cv::imwrite("F.jpg", mROIFL_F);
	cv::imshow("L", mROIFL_L);
	cv::imwrite("L.jpg", mROIFL_L);
	cv::waitKey(0);


	return 0;
}