
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <iostream>
#include <vector>
using namespace std;





void ExtractFeatures(cv::Mat mImage1, cv::Mat mImage2, vector<cv::DMatch> & gMatches,
						vector<cv::KeyPoint> & gKeyPoints1, vector<cv::KeyPoint> & gKeyPoints2){
    
	gKeyPoints1.clear();
	gKeyPoints2.clear();
    cv::Mat mDescriptors1, mDescriptors2;

    // cout << "Extract Features" << endl;
    cv::Ptr<cv::FeatureDetector> pDetector = cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::DescriptorExtractor> pDescriptor = cv::xfeatures2d::SIFT::create();
    // cv::Ptr<cv::DescriptorMatcher> pMatcher  = cv::DescriptorMatcher::create ( "BruteForce" );
    // cout << "Finish Extract Features" << endl;

    cv::Ptr<cv::BFMatcher> pMatcher = cv::BFMatcher::create(cv::NORM_L2 , true);
    
    // cv::imshow("const cv::String &winname", mImage1);
    // cv::waitKey(0);
    // cv::imshow("const cv::String &winname", mImage2);
    // cv::waitKey(0);
    // cout << "Finish Show" << endl;

    //Detect the corner
    pDetector->detect(mImage1, gKeyPoints1);
    pDetector->detect(mImage2, gKeyPoints2);
    // cout << "Finish detect" << endl;

    //Compute the descriptor
    pDescriptor->compute(mImage1, gKeyPoints1, mDescriptors1);
    pDescriptor->compute(mImage2, gKeyPoints2, mDescriptors2);
    cout  << "Finish compute" << endl;

    cout << "Descriptor size " << mDescriptors1.size() << endl;

    cout << "Descriptor size " << mDescriptors2.size() << endl;

    if (mDescriptors1.size().height <2 || mDescriptors2.size().height <2){
    	return;
    }

    pMatcher->match(mDescriptors1, mDescriptors2, gMatches);
    cout << "Finish match" << endl;
    cout << "gMatches size" << gMatches.size() << endl;
    // 仅供娱乐的写法
    double nMinDist = min_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    double nMaxDist = max_element( gMatches.begin(), gMatches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    sort(gMatches.begin(),gMatches.end() , 
    	[](cv::DMatch iMatch1 , cv::DMatch iMatch2){ return iMatch1.distance < iMatch2.distance;});
 


    // cv::Mat mGoodImage;
    // cv::drawMatches ( mImage1, gKeyPoints1, mImage2, gKeyPoints2, gMatches, mGoodImage );
    // cv::imshow("const cv::String &winname", mGoodImage);
    // cv::waitKey(0);


}


void GeneratePointsPair(cv::Mat mBirdseyeView_1, cv::Mat mBirdseyeView_2,
						vector<Eigen::Vector2d> & gPoints1, vector<Eigen::Vector2d> & gPoints2){
	vector<cv::DMatch> gMatches;
	vector<cv::KeyPoint> gKeyPoints1, gKeyPoints2;
	//Extract features and then match them.
	ExtractFeatures(mBirdseyeView_1, mBirdseyeView_2, gMatches,
					gKeyPoints1, gKeyPoints2);
	cout << "Finish ExtractFeatures" << endl;

	cv::Mat mConcat = mBirdseyeView_1.clone();
	// cv::vconcat(mConcat, mBirdseyeView_2 , mConcat);
	

	for (cv::DMatch iMatch : gMatches){
		cv::Point2f iPoint1 = gKeyPoints1[iMatch.queryIdx].pt;
		cv::Point2f iPoint2 = gKeyPoints2[iMatch.trainIdx].pt;
		
		//Use a threshold to eliminate inliers.
		double nDistance = (iPoint1.x - iPoint2.x) * (iPoint1.x - iPoint2.x);
		nDistance += (iPoint1.y - iPoint2.y) * (iPoint1.y - iPoint2.y);
		// cout << "Distance is " << nDistance << endl;
		if (nDistance >= 1000){
			continue;
		}
		// cv::line(mConcat, iPoint1, cv::Point2f(iPoint2.x , iPoint2.y + 400),
		// 		 cv::Scalar(100 , 0 , 0));
		//Relative to left-top
		// gPoints1.push_back(
		// 	Eigen::Vector2d(iPoint1.x + nStartX, iPoint1.y)
		// 	);
		// gPoints2.push_back(
		// 	Eigen::Vector2d(iPoint2.x + nStartY, iPoint2.y)
		// 	);
		gPoints1.push_back(
			Eigen::Vector2d(iPoint1.x, iPoint1.y)
			);
		gPoints2.push_back(
			Eigen::Vector2d(iPoint2.x, iPoint2.y)
			);
	}

	// cv::imshow("const cv::String &winname", mConcat);
	// cv::waitKey(0);

	
}



void OptimizeHomographyPose(	vector<Eigen::Vector2d> & gPoints1,
								vector<Eigen::Vector2d> & gPoints2,
								cv::Mat & mPose_CV){
	vector<cv::Point2f> gPoints1_CV, gPoints2_CV;
	for (int i=0;i<gPoints2.size();i++){
		Eigen::Vector2d iPoint1 = gPoints1[i];
		Eigen::Vector2d iPoint2 = gPoints2[i];
		gPoints1_CV.push_back(cv::Point2f(iPoint1(0) , iPoint1(1)));
		gPoints2_CV.push_back(cv::Point2f(iPoint2(0) , iPoint2(1)));

		// cout << "Point1 " << gPoints1[i] << endl;
		// cout << "Point2 " << gPoints2[i] << endl;
	}

	cv::Mat mHomography = cv::findHomography(gPoints1_CV, gPoints2_CV , cv::RANSAC ,5);
	// cout << "mHomography is " << endl << mHomography << endl;
	mPose_CV = mHomography.clone();

}


void CopyImage(cv::Mat & mFromImage,
			   cv::Mat & mGeneratedImage,
			   cv::Mat & mPose_CV){
	cout << "Homo is " << endl << mPose_CV << endl;
	if (mPose_CV.size().width != 3){
		mGeneratedImage = mFromImage.clone();
		return;
	}
    mGeneratedImage = mFromImage.clone();
    for (int u=0;u<mGeneratedImage.cols;u++){
    	for (int v=0;v<mGeneratedImage.rows;v++){
    		
    		cv::Mat mPos = (cv::Mat_<double>(3 , 1) << u , v , 1);
    		
    		mPos = mPose_CV *  mPos;
    		 int nNewU = mPos.at<double>(0 , 0);
    		int nNewV =  mPos.at<double>(1 , 0);
    		if (nNewU <= 1 || nNewV <=1 || 
    			nNewU >= mGeneratedImage.cols-1 ||
    			nNewV >= mGeneratedImage.rows-1){
    			mGeneratedImage.at<cv::Vec3b>(v , u) = cv::Vec3b(0 , 0 , 0);
    		}else{
    			mGeneratedImage.at<cv::Vec3b>(v , u) = mFromImage.at<cv::Vec3b>(nNewV, nNewU);
    		}
    	}
    }

    // cv::imshow("mFromImage", mFromImage);
    // cv::imwrite("from.jpg", mFromImage);
    // cv::waitKey(0);
    // cv::imshow("mFromImage", mGeneratedImage);
    // cv::imwrite("generated.jpg", mGeneratedImage);
    // cv::waitKey(0);

}




double CalculateSigma(vector<cv::Vec3b> iPointVec1 ,vector<cv::Vec3b> iPointVec2){
		
		int nSize = 0;
		double nTotalSigma = 0.0;
		int nTotalSize = iPointVec1.size();
		
		for (int i=0;i<nTotalSize;i++){

			cv::Vec3b mColorPointA = iPointVec1[i];
			cv::Vec3b mColorPointB = iPointVec2[i];
			if (mColorPointB[0] == 0 && 
					mColorPointB[1] == 0 && 
					mColorPointB[2] == 0){
					continue;
			}

			if (mColorPointB[0] == 0){
				mColorPointB[0] = 1;
			}

			if (mColorPointB[1] == 0){
				mColorPointB[1] = 1;
			}

			if (mColorPointB[2] == 0){
				mColorPointB[2] = 1;
			}

			float nScale1 = (float)mColorPointA[0]/(float)mColorPointB[0];
			float nScale2 = (float)mColorPointA[1]/(float)mColorPointB[1];
			float nScale3 = (float)mColorPointA[2]/(float)mColorPointB[2];
			float nAverageScale = (nScale1 + nScale2 + nScale3)/3;
			float nSigma = sqrt((nScale1 - nAverageScale) * (nScale1 - nAverageScale) + 
								(nScale2 - nAverageScale) * (nScale2 - nAverageScale) + 
								(nScale3 - nAverageScale) * (nScale3 - nAverageScale));
			nTotalSigma += nSigma;
		}

		if (nTotalSize == 0){
			return -1.0;
		}
				

		return nTotalSigma / nTotalSize;

		
}





double CalculateSigma(cv::Mat mImage1 , cv::Mat mGeneratedImage, cv::Point2f iPoint){
		int u = iPoint.x;
		int v = iPoint.y;

		int nCol = mImage1.cols;
		int nRow = mImage1.rows;

		int nSize = 0;
		double nTotalSigma = 0.0;
		for (int i = -6;i<=6;i+=3){
			for (int j=-6;j<=6;j+=3){
				if (u + i <= 0 || v + j <=0 || u+i >= nCol || v+j >= nRow){
					continue;
				}
				cv::Vec3b mColorPointA = mImage1.at<cv::Vec3b>(v+i , u+i);
				cv::Vec3b mColorPointB = mGeneratedImage.at<cv::Vec3b>(v+i , u+i);

				if (mColorPointB[0] == 0 && 
					mColorPointB[1] == 0 && 
					mColorPointB[2] == 0){
					continue;
				}

				if (mColorPointB[0] == 0){
					mColorPointB[0] = 1;
				}

				if (mColorPointB[1] == 0){
					mColorPointB[1] = 1;
				}

				if (mColorPointB[2] == 0){
					mColorPointB[2] = 1;
				}

				float nScale1 = (float)mColorPointA[0]/(float)mColorPointB[0];
				float nScale2 = (float)mColorPointA[1]/(float)mColorPointB[1];
				float nScale3 = (float)mColorPointA[2]/(float)mColorPointB[2];
				float nAverageScale = (nScale1 + nScale2 + nScale3)/3;
				float nSigma = sqrt((nScale1 - nAverageScale) * (nScale1 - nAverageScale) + 
									(nScale2 - nAverageScale) * (nScale2 - nAverageScale) + 
									(nScale3 - nAverageScale) * (nScale3 - nAverageScale));
				nTotalSigma += nSigma;
				nSize++;
			}
		}


		if (nSize == 0){
			return -1.0;
		}
				

		return nTotalSigma / nSize;

		
}

// vector<cv::Point2d> Culling(vector<cv::Point2d> gPoints, cv::Mat &mImage1, cv::Mat &mImage2 , cv::Mat & mHomography , cv::Rect iROI){
// 	vector<cv::Point2d> gResPoints;
// 	gResPoints.reserve(gPoints.size());

// 	vector<float> gSigma;
// 	for (auto iPoint : gPoints){

// 		int u = iPoint.x;
// 		int v = iPoint.y;
// 		cv::Mat mPos = (cv::Mat_<double>(3 , 1) << u , v , 1);
		
// 		mPos = mHomography *  mPos;
// 		int nNewU = mPos.at<double>(0 , 0);
// 		int nNewV =  mPos.at<double>(1 , 0);
// 		if (nNewU <= 1 || nNewV <=1 || 
// 			nNewU >= mImage2.cols-1 ||
// 			nNewV >= mImage2.rows-1){
// 			gSigma.push_back(-1);
// 		}else{
// 			gSigma.push_back(CalculateSigma(mImage1.at<cv::Vec3b>(v , u),
// 			                                mImage2.at<cv::Vec3b>(nNewV, nNewU)));
// 		}

// 	}


// 	double nTotal, nMeanSigma;
// 	//Get the mean gSigma.
// 	double nSig = 0.0;

// 	int nNum = 0;
// 	for (auto item : gSigma){
// 		if (item < 1 && item >=0.0){
// 			nTotal += item;
// 			nNum++;
// 		}
// 	}
// 	nMeanSigma = nTotal /(float)nNum;

// 	for (auto item : gSigma){
// 		if (item < 1 && item >=0.0){
// 			nSig += ((item-nMeanSigma) * (item-nMeanSigma))/(float)nNum;
// 		}
// 	}
// 	nSig = sqrt(nSig);

// 	for (int i=0;i<gSigma.size();i++){
// 		float nSigma = gSigma[i];
// 		cv::Point2d iPoint = gPoints[i];
// 		if (nSigma < nMeanSigma || nSigma < 0){
// 			gResPoints.push_back(gPoints[i]);
// 		}
// 	}
// 	return gResPoints;


// }



vector<cv::Point2d> Culling(vector<cv::Point2d> gPoints, cv::Mat & mImage1 , cv::Mat & mImage2){

	
	vector<cv::Point2d> gResPoints;
	gResPoints.reserve(gPoints.size());

	vector<float> gSigma;
	for (auto iPoint : gPoints){
		double nSigma = CalculateSigma(mImage1, mImage2, iPoint);
		
		gSigma.push_back(nSigma);
		
	}


	double nTotal, nMeanSigma;
	//Get the mean gSigma.
	double nSig = 0.0;

	int nNum = 0;
	for (auto item : gSigma){
		if (item < 1  && item >=0){
			nTotal += item;
			nNum++;
		}
	}
	nMeanSigma = nTotal /(float)nNum;

	for (auto item : gSigma){
		if (item < 1 && item >=0){
			nSig += ((item-nMeanSigma) * (item-nMeanSigma))/(float)nNum;
		}
	}
	nSig = sqrt(nSig);

	cv::Mat mCull = mImage1.clone();
	cv::Mat mUnCull = mCull.clone();

	for (int i=0;i<gSigma.size();i++){
		float nSigma = gSigma[i];
		cv::Point2d iPoint = gPoints[i];
		if (nSigma < nMeanSigma){
			gResPoints.push_back(gPoints[i]);
			cv::circle(mCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
			cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
		}else{
			cv::circle(mUnCull, iPoint, 3, cv::Scalar(100 , 0 , 0));
		}
	}



	// cv::imshow("from", mImage1);
	
	// cv::waitKey(0);
	// cv::imshow("from", mImage2);
	
	// cv::waitKey(0);

	// cv::imshow("from", mUnCull);
	
	// cv::waitKey(0);
	// cv::imshow("cull", mCull);
	
	// cv::waitKey(0);

	return gResPoints;
	
}