#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"
#include <iostream>

using namespace std;








void ExtractFeatures(cv::Mat mImage1, cv::Mat mImage2, vector<cv::DMatch> & gMatches,
						vector<cv::KeyPoint> & gKeyPoints1, vector<cv::KeyPoint> & gKeyPoints2){
    
    cv::Mat mDescriptors1, mDescriptors2;



    // cv::Ptr<cv::FeatureDetector> pDetector = cv::xfeatures2d::SIFT::create();
    // cv::Ptr<cv::DescriptorExtractor> pDescriptor = cv::xfeatures2d::SIFT::create();



    cv::Ptr<cv::FeatureDetector> pDetector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> pDescriptor = cv::ORB::create();

    // cv::Ptr<cv::DescriptorMatcher> pMatcher  = cv::DescriptorMatcher::create ( "BruteForce" );

    cv::Ptr<cv::BFMatcher> pMatcher = cv::BFMatcher::create(cv::NORM_L2 , true);
    

    //Detect the corner
    pDetector->detect(mImage1, gKeyPoints1);
    pDetector->detect(mImage2, gKeyPoints2);
    cout << "Finish detect" << endl;

    //Compute the descriptor
    pDescriptor->compute(mImage1, gKeyPoints1, mDescriptors1);
    pDescriptor->compute(mImage2, gKeyPoints2, mDescriptors2);
    cout  << "Finish compute" << endl;

    cout << "Descriptor size " << mDescriptors1.size() << endl;

    cout << "Descriptor size " << mDescriptors2.size() << endl;

    pMatcher->match(mDescriptors1, mDescriptors2, gMatches);
    cout << "Finish match" << endl;
    cout << "gMatches size" << gMatches.size() << endl;
    // 仅供娱乐的写法
    double nMinDist = min_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    double nMaxDist = max_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    sort(gMatches.begin(),gMatches.end() , 
    	[](cv::DMatch iMatch1 , cv::DMatch iMatch2){ return iMatch1.distance < iMatch2.distance;});
    
    cout << "MinDist is " << nMinDist << endl;
    cout << "MaxDist is " << nMaxDist << endl;



    vector<cv::DMatch> gGoodMatches;
    for (int i=0;i<30;i++){
    	gGoodMatches.push_back(gMatches[i]);
    }

    cv::Mat mGoodImage;
    cv::drawMatches ( mImage1, gKeyPoints1, mImage2, gKeyPoints2, gGoodMatches, mGoodImage );
    cv::imshow("const cv::String &winname", mGoodImage);
    cv::waitKey(0);


}






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
	vector<SVPair> gPairs = iLoader.LoadFramePairs(vector<int>{0 , 1 , 2 , 3 , 4});

	SurroundView iSurround(pFrontCamera, pLeftCamera, pBackCamera, pRightCamera);

	cout << "Bind images" << endl;
	iSurround.BindImagePairs(gPairs);

	cout << "Init K_G" << endl;
	iSurround.InitK_G(1000, 1000, 0.1, 0.1);

	cout << "Finish init K_G" << endl;



	cv::Mat mFrontFisheye_Front , mFrontFisheye_Left, mFrontFisheye_Back, mFrontFisheye_Right;
	mFrontFisheye_Front = gPairs[3].m_pFrontFrame->m_mFisheyeImage;
	mFrontFisheye_Left = gPairs[3].m_pLeftFrame->m_mFisheyeImage;
	mFrontFisheye_Back = gPairs[3].m_pBackFrame->m_mFisheyeImage;
	mFrontFisheye_Right = gPairs[3].m_pRightFrame->m_mFisheyeImage;



	cv::Rect iRectFront = cv::Rect(100 , 100 , 400 , 500);
	cv::Rect iRectLeft = cv::Rect( 780 , 100 , 400 , 500);


	cv::imshow("const cv::String &winname", mFrontFisheye_Front);
	cv::imshow("ROI", mFrontFisheye_Front(iRectFront));
	cv::waitKey(0);
	cv::imshow("const cv::String &winname", mFrontFisheye_Left);
	cv::imshow("ROI", mFrontFisheye_Left(iRectLeft));
	cv::waitKey(0);
	cv::imshow("const cv::String &winname", mFrontFisheye_Back);
	cv::waitKey(0);
	cv::imshow("const cv::String &winname", mFrontFisheye_Right);
	cv::waitKey(0);

	cv::Mat mFrontROI = mFrontFisheye_Front(iRectFront);
	cv::Mat mLeftROI = mFrontFisheye_Left(iRectLeft);



	vector<cv::DMatch> gMatches;
	vector<cv::KeyPoint> gKeyPoints1, gKeyPoints2;
	ExtractFeatures(mFrontROI, mLeftROI, gMatches,
					gKeyPoints1, gKeyPoints2);



	return 0;
}