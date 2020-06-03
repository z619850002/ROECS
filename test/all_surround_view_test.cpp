#include "../include/surround/surround_view_system.h"
#include "../include/loader/frame_loader.h"
#include "../include/initializer/initializer.h"
#include <sstream>
#include <fstream>
#include <ostream>


using namespace std;


cv::Mat eigen2mat(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A)
{
	cv::Mat B;
	cv::eigen2cv(A,B);
	
	return B;
}


double CalculatePhotometricError(cv::Mat & mImage_1 , cv::Mat & mImage_2 , cv::Rect iROI){
	cv::Mat mGrayImage_1 , mGrayImage_2;
	cv::cvtColor(mImage_1(iROI),mGrayImage_1,cv::COLOR_BGR2GRAY);
	cv::cvtColor(mImage_2(iROI),mGrayImage_2,cv::COLOR_BGR2GRAY);	
	mGrayImage_1.convertTo(mGrayImage_1, CV_64FC1);
	mGrayImage_2.convertTo(mGrayImage_2, CV_64FC1);

	cv::Mat mDiff;
	cv::subtract(mGrayImage_1, mGrayImage_2, mDiff);

	return cv::norm(mDiff , cv::NORM_L2);
}



cv::Mat project_on_ground(cv::Mat img, Sophus::SE3 T_CG,
						  Eigen::Matrix3d K_C,Eigen::Vector4d D_C,
						  cv::Mat K_G,int rows, int cols)
{
// 	cout<<"--------------------Init p_G and P_G------------------------"<<endl;
	cv::Mat p_G = cv::Mat::ones(3,rows*cols,CV_64FC1);
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			p_G.at<double>(0,cols*i+j) = j;
			p_G.at<double>(1,cols*i+j) = i;
		}
	}
	
	cv::Mat P_G = cv::Mat::ones(4,rows*cols,CV_64FC1);
	P_G(cv::Rect(0,0,rows*cols,3)) = K_G.inv()*p_G;
	P_G(cv::Rect(0,2,rows*cols,1)) = 0;
	
// 	cout<<"--------------------Init P_GF------------------------"<<endl;

	cv::Mat P_GC = cv::Mat::zeros(4,rows*cols,CV_64FC1);
	cv::Mat T_CG_(4,4,CV_64FC1);
	cv::eigen2cv(T_CG.matrix(),T_CG_);
	P_GC =  T_CG_ * P_G;

	
	cv::Mat P_GC1 = cv::Mat::zeros(1,rows*cols,CV_64FC2);
	vector<cv::Mat> channels(2);
	cv::split(P_GC1, channels);
	channels[0] = P_GC(cv::Rect(0,0,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
	channels[1] = P_GC(cv::Rect(0,1,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
	cv::merge(channels, P_GC1);

	cv::Mat p_GC = cv::Mat::zeros(1,rows*cols,CV_64FC2);
// 	cout<<eigen2mat(K_C)<<endl;
	vector<double> D_C_{D_C(0,0),D_C(1,0),D_C(2,0),D_C(3,0)};
	cv::fisheye::distortPoints(P_GC1,p_GC,eigen2mat(K_C),D_C_);
// 	cout<<"p_GC: "<<endl;
	p_GC.reshape(rows,cols);
	cv::Mat p_GC_table = p_GC.reshape(0,rows);
	vector<cv::Mat> p_GC_table_channels(2);
	cv::split(p_GC_table, p_GC_table_channels);
	
	cv::Mat p_GC_table_32F;
	p_GC_table.convertTo(p_GC_table_32F,CV_32FC2);
	
	cv::Mat img_GC;
	cv::remap(img,img_GC,p_GC_table_32F,cv::Mat(),cv::INTER_LINEAR);
	
	return img_GC;
}




int main(int argc , char * argv[]){


	//ROI
	int ROI_FL_x = 200;
	int ROI_FL_y = 0;
	int ROI_FL_w = 200;
	int ROI_FL_h = 200;
	
	int ROI_LB_x = 200;
	int ROI_LB_y = 800;
	int ROI_LB_w = 200;
	int ROI_LB_h = 200;
	
	int ROI_BR_x = 650;
	int ROI_BR_y = 800;
	int ROI_BR_w = 200;
	int ROI_BR_h = 200;
	
	int ROI_RF_x = 650;
	int ROI_RF_y = 0;
	int ROI_RF_w = 200;
	int ROI_RF_h = 200;
	
	cv::Rect iROI_FL = cv::Rect(ROI_FL_x-50,ROI_FL_y,ROI_FL_w,ROI_FL_h+70);
	cv::Rect iROI_LB = cv::Rect(ROI_LB_x - 50,ROI_LB_y-100,ROI_LB_w,ROI_LB_h+70);
	cv::Rect iROI_BR = cv::Rect(ROI_BR_x,ROI_BR_y-100,ROI_BR_w,ROI_BR_h+70);
	cv::Rect iROI_RF = cv::Rect(ROI_RF_x,ROI_RF_y,ROI_RF_w + 100,ROI_RF_h+70);





















	Initializer iInitializer;
	Camera * pFrontCamera, * pLeftCamera, * pBackCamera, * pRightCamera;
	iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


	pLeftCamera->BlurPose();
	pRightCamera->BlurPose();


	FrameLoader iLoader("/home/kyrie/Documents/Data/204-315" , pFrontCamera,
						pLeftCamera, pBackCamera, 
						pRightCamera);

	cout << "Load pairs" << endl;
	vector<int> gIndices = {};
	for (int i=0;i<100;i++){
		gIndices.push_back(i);
	}
	vector<SVPair> gPairs = iLoader.LoadFramePairs(gIndices);



	int nBeginIndex = std::stoi(argv[1]);
	int nEndIndex = std::stoi(argv[2]);
	// cout << "Index is " << nSampleIndex << endl;




	for (int nImageIndex = nBeginIndex; nImageIndex < nEndIndex; nImageIndex++){
		

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

		ofstream iOutFile("./ROECS/" + aImageIndex +  "_pose.txt");
		ofstream iOutFile2("./ROECS/" + aImageIndex +  "_pose2.txt");



		Sophus::SE3 sPose_Front = pFrontCamera->m_mT;
			Sophus::SE3 sPose_Left = pLeftCamera->m_mT;
			Sophus::SE3 sPose_Back = pBackCamera->m_mT;
			Sophus::SE3 sPose_Right = pRightCamera->m_mT;


			SVPair iPair = iSurround.m_gDistortedPairs[nImageIndex];

			cv::Mat mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 sPose_Front,
											 iSurround.m_pFrontCamera->m_mK, 
											 iSurround.m_pFrontCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			cv::Mat mBirdsLeft = project_on_ground( iPair.m_pLeftFrame->m_mFisheyeImage,
											 sPose_Left,
											 iSurround.m_pLeftCamera->m_mK, 
											 iSurround.m_pLeftCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			cv::Mat mBirdsBack = project_on_ground( iPair.m_pBackFrame->m_mFisheyeImage,																																																																																																																	
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																								 sPose_Back,
											 iSurround.m_pBackCamera->m_mK, 
											 iSurround.m_pBackCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);
																																																																					
			cv::Mat mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
											 sPose_Right,
											 iSurround.m_pRightCamera->m_mK, 
											 iSurround.m_pRightCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);


			double nTotalError = CalculatePhotometricError(mBirdsLeft , mBirdsFront , iROI_FL);
			nTotalError += CalculatePhotometricError(mBirdsBack , mBirdsLeft , iROI_LB);
			nTotalError += CalculatePhotometricError(mBirdsRight , mBirdsBack , iROI_BR);
			nTotalError += CalculatePhotometricError(mBirdsFront , mBirdsRight , iROI_RF);

			iOutFile << "Error in iteration " << 0 << " is : " << nTotalError << endl;
			cout << "Error in iteration " << 0 << " is : " << nTotalError << endl;


			iPair = iSurround.m_gDistortedPairs[99 - nImageIndex];


			mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 sPose_Front,
											 iSurround.m_pFrontCamera->m_mK, 
											 iSurround.m_pFrontCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			mBirdsLeft = project_on_ground( iPair.m_pLeftFrame->m_mFisheyeImage,
											 sPose_Left,
											 iSurround.m_pLeftCamera->m_mK, 
											 iSurround.m_pLeftCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			mBirdsBack = project_on_ground( iPair.m_pBackFrame->m_mFisheyeImage,
											 sPose_Back,
											 iSurround.m_pBackCamera->m_mK, 
											 iSurround.m_pBackCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
											 sPose_Right,
											 iSurround.m_pRightCamera->m_mK, 
											 iSurround.m_pRightCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			nTotalError = CalculatePhotometricError(mBirdsLeft , mBirdsFront , iROI_FL);
			nTotalError += CalculatePhotometricError(mBirdsBack , mBirdsLeft , iROI_LB);
			nTotalError += CalculatePhotometricError(mBirdsRight , mBirdsBack , iROI_BR);
			nTotalError += CalculatePhotometricError(mBirdsFront , mBirdsRight , iROI_RF);

			iOutFile2 << "Error in iteration " << 0 << " is : " << nTotalError << endl;
			cout << "Error2 in iteration " << 0 << " is : " << nTotalError << endl;





















		cv::Mat mSurroundView = iSurround.GenerateSurroundView(nImageIndex, 1000, 1000);
		cv::imwrite("./ROECS/" + aImageIndex +  "_before.jpg" , mSurroundView);


		cv::Mat mSurroundView2 = iSurround.GenerateSurroundView(99 - nImageIndex, 1000, 1000);
		cv::imwrite("./ROECS/" + aImageIndex +  "_before2.jpg" , mSurroundView2);



		// cv::imshow("const cv::String &winname", mSurroundView);
		// cv::waitKey(30);
		

		// iSurround.OptimizePoseWithOneFrame(nImageIndex);

		vector<int> gOptimizeIndices = {};
		for (int i=0;i<=60;i+=6){			
			gOptimizeIndices.push_back((nImageIndex + i)%99);
		}
		// iSurround.OptimizeWithMultiFrame(gOptimizeIndices);
		vector<vector<Sophus::SE3>> gPoses = iSurround.OptimizeWithCulling(gOptimizeIndices, iOutFile);

		mSurroundView = iSurround.GenerateSurroundView(nImageIndex, 1000, 1000);
		cv::imwrite("./ROECS/" + aImageIndex +  "_after.jpg" , mSurroundView);

		mSurroundView2 = iSurround.GenerateSurroundView(99 - nImageIndex, 1000, 1000);
		cv::imwrite("./ROECS/" + aImageIndex +  "_after2.jpg" , mSurroundView2);
		// cv::imshow("const cv::String &winname", mSurroundView);
		// cv::waitKey(30);


			


		//Calculate the photometric error.
		for (int i=0;i<gPoses[0].size();i++){
			Sophus::SE3 sPose_Front = gPoses[0][i];
			Sophus::SE3 sPose_Left = gPoses[1][i];
			Sophus::SE3 sPose_Back = gPoses[2][i];
			Sophus::SE3 sPose_Right = gPoses[3][i];


			SVPair iPair = iSurround.m_gDistortedPairs[nImageIndex];

			cv::Mat mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 sPose_Front,
											 iSurround.m_pFrontCamera->m_mK, 
											 iSurround.m_pFrontCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			cv::Mat mBirdsLeft = project_on_ground( iPair.m_pLeftFrame->m_mFisheyeImage,
											 sPose_Left,
											 iSurround.m_pLeftCamera->m_mK, 
											 iSurround.m_pLeftCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			cv::Mat mBirdsBack = project_on_ground( iPair.m_pBackFrame->m_mFisheyeImage,																																																																																																																	
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																								 sPose_Back,
											 iSurround.m_pBackCamera->m_mK, 
											 iSurround.m_pBackCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);
																																																																					
			cv::Mat mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
											 sPose_Right,
											 iSurround.m_pRightCamera->m_mK, 
											 iSurround.m_pRightCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);


			double nTotalError = CalculatePhotometricError(mBirdsLeft , mBirdsFront , iROI_FL);
			nTotalError += CalculatePhotometricError(mBirdsBack , mBirdsLeft , iROI_LB);
			nTotalError += CalculatePhotometricError(mBirdsRight , mBirdsBack , iROI_BR);
			nTotalError += CalculatePhotometricError(mBirdsFront , mBirdsRight , iROI_RF);

			iOutFile << "Error in iteration " << i+1 << " is : " << nTotalError << endl;
			cout << "Error in iteration " << i+1 << " is : " << nTotalError << endl;


			iPair = iSurround.m_gDistortedPairs[99 - nImageIndex];

			mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 sPose_Front,
											 iSurround.m_pFrontCamera->m_mK, 
											 iSurround.m_pFrontCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			mBirdsLeft = project_on_ground( iPair.m_pLeftFrame->m_mFisheyeImage,
											 sPose_Left,
											 iSurround.m_pLeftCamera->m_mK, 
											 iSurround.m_pLeftCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			mBirdsBack = project_on_ground( iPair.m_pBackFrame->m_mFisheyeImage,
											 sPose_Back,
											 iSurround.m_pBackCamera->m_mK, 
											 iSurround.m_pBackCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
											 sPose_Right,
											 iSurround.m_pRightCamera->m_mK, 
											 iSurround.m_pRightCamera->m_mD,
											 iSurround.m_mK_G, 1000, 1000);

			nTotalError = CalculatePhotometricError(mBirdsLeft , mBirdsFront , iROI_FL);
			nTotalError += CalculatePhotometricError(mBirdsBack , mBirdsLeft , iROI_LB);
			nTotalError += CalculatePhotometricError(mBirdsRight , mBirdsBack , iROI_BR);
			nTotalError += CalculatePhotometricError(mBirdsFront , mBirdsRight , iROI_RF);

			iOutFile2 << "Error in iteration " << i+1 << " is : " << nTotalError << endl;
			cout << "Error2 in iteration " << i+1 << " is : " << nTotalError << endl;



		}







		iInitializer.InitializeCameras(	pFrontCamera,
									pLeftCamera,
									pBackCamera,
									pRightCamera);


		pLeftCamera->BlurPose();
		pRightCamera->BlurPose();
	
	}

	

	return 0;
}