#include "../../include/surround/surround_view_system.h"

using namespace std;

cv::Mat eigen2mat(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A)
{
	cv::Mat B;
	cv::eigen2cv(A,B);
	
	return B;
}

void imshow_64F(cv::Mat img,string img_name)
{
	cv::Mat img_norm,img_color;
	cv::normalize(img,img_norm,0,255,cv::NORM_MINMAX,CV_8U);
	cv::applyColorMap(img_norm,img_color, cv::COLORMAP_JET);
	cv::namedWindow(img_name,0);
	cv::imshow(img_name,img_color);
	
	return;
}

void imshow_64F_gray(cv::Mat img,string img_name)
{
	cv::Mat img_8U;
	img.convertTo(img_8U,CV_8U);
	cv::namedWindow(img_name,0);
	cv::imshow(img_name,img_8U);
	
	return;
}

cv::Mat bilinear_interpolation(cv::Mat img,cv::Mat pix_table,int rows, int cols)
{
	cv::Mat img_G(rows,cols,CV_8UC3);
	// f is float
	cv::Mat img_G_f(rows,cols,CV_64FC3);
	cv::Mat img_f;
	img.convertTo(img_f, CV_64FC3);
	float x;
	float y;
	int x_floor;
	int y_floor;
	int x_ceil;
	int y_ceil;
	cv::Vec3d ul;
	cv::Vec3d ur;
	cv::Vec3d dl;
	cv::Vec3d dr;
	cv::Vec3d pix;
	
	
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			x = pix_table.at<cv::Vec2d>(i,j)[1];
			y = pix_table.at<cv::Vec2d>(i,j)[0];
			if(x<0 || y<0 || (y>=img.cols-1) || (x>=img.rows-1))
			{
				img_G_f.at<cv::Vec3d>(i,j) = cv::Vec3d(0,0,0);
			}
			else{
				x_floor = int(floor(x));
				x_ceil = x_floor+1;
				y_floor = int(floor(y));
				y_ceil = y_floor+1;
				
				ul = img_f.at<cv::Vec3d>(x_floor,y_floor);
				ur = img_f.at<cv::Vec3d>(x_ceil,y_floor);
				dl = img_f.at<cv::Vec3d>(x_floor,y_ceil);
				dr = img_f.at<cv::Vec3d>(x_ceil,y_ceil);
				pix = (ur*(x-x_floor)+ul*(x_ceil-x))*(y_ceil-y)
					 +(dr*(x-x_floor)+dl*(x_ceil-x))*(y-y_floor);
				if(pix(0)>=0   && pix(1)>=0   && pix(2)>=0 && 
				   pix(0)<=255 && pix(1)<=255 && pix(2)<=255)
				{
					img_G_f.at<cv::Vec3d>(j,i) = pix;
				}
			}
		}
	}
	
	img_G_f.convertTo(img_G, CV_8UC3);
	
	return img_G;
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

	
// 	cout<<"--------------------Init P_GF1------------------------"<<endl;
	cv::Mat P_GC1 = cv::Mat::zeros(1,rows*cols,CV_64FC2);
	vector<cv::Mat> channels(2);
	cv::split(P_GC1, channels);
	channels[0] = P_GC(cv::Rect(0,0,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
	channels[1] = P_GC(cv::Rect(0,1,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
	cv::merge(channels, P_GC1);
// 	cout<<"P_GC1: "<<endl;
// 	cout<<P_GC1(cv::Rect(0,0,5,1))<<endl<<endl;
	
// 	cout<<"--------------------Init p_GF------------------------"<<endl;
// 	cout<<K_C.cols()<<endl;
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
// 	img_GC = bilinear_interpolation(img,p_GC_table,rows,cols);
// 	cout<<img_GC.size<<endl;
	
	return img_GC;
}


cv::Mat generate_surround_view(cv::Mat img_GF, cv::Mat img_GL, 
							   cv::Mat img_GB, cv::Mat img_GR, 
							   int rows, int cols)
{
	cv::Mat img_G(rows,cols,CV_8UC3);
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			if(i>2*j-500)
			{
				if(i>-2*j+1500)
				{
					img_G.at<cv::Vec3b>(i,j) = img_GB.at<cv::Vec3b>(i,j);
				}
				else
				{
					img_G.at<cv::Vec3b>(i,j) = img_GL.at<cv::Vec3b>(i,j);
				}
				
			}
			else
			{
				if(i>-2*j+1500)
				{
					img_G.at<cv::Vec3b>(i,j) = img_GR.at<cv::Vec3b>(i,j);
				}
				else
				{
					img_G.at<cv::Vec3b>(i,j) = img_GF.at<cv::Vec3b>(i,j);
				}
			}
			
		}
	}
	
	for(int i=300;i<700;i++)
	{
		for(int j=400;j<600;j++)
		{
			img_G.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
		}
	}
	
	return img_G;
}



SurroundView::SurroundView(){

}



SurroundView::SurroundView(	Camera * pFrontCamera, Camera * pLeftCamera, 
					Camera * pBackCamera, Camera * pRightCamera):
	m_pFrontCamera(pFrontCamera), m_pLeftCamera(pLeftCamera),
	m_pBackCamera(pBackCamera), m_pRightCamera(pRightCamera)
{

}


cv::Mat SurroundView::GenerateSurroundView(int nIndex, int nRows, int nCols){
	if (nIndex >= this->m_gDistortedPairs.size()){
		cout << "Pairs out of index, please check the index of the surround view inputed.";
	}

	SVPair iPair = this->m_gDistortedPairs[nIndex];

	//Get the bird's-eye view image.


	cv::Mat mBirdsFront = project_on_ground( iPair.m_pFrontFrame->m_mFisheyeImage,
											 this->m_pFrontCamera->m_mT,
											 this->m_pFrontCamera->m_mK, 
											 this->m_pFrontCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	cv::Mat mBirdsLeft = project_on_ground(  iPair.m_pLeftFrame->m_mFisheyeImage,
											 this->m_pLeftCamera->m_mT,
											 this->m_pLeftCamera->m_mK, 
											 this->m_pLeftCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	cv::Mat mBirdsBack = project_on_ground(  iPair.m_pBackFrame->m_mFisheyeImage,
											 this->m_pBackCamera->m_mT,
											 this->m_pBackCamera->m_mK, 
											 this->m_pBackCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	cv::Mat mBirdsRight = project_on_ground( iPair.m_pRightFrame->m_mFisheyeImage,
											 this->m_pRightCamera->m_mT,
											 this->m_pRightCamera->m_mK, 
											 this->m_pRightCamera->m_mD,
											 this->m_mK_G, nRows, nCols);

	
	
	//Stitch the bird's-eye view image to get the surround-view image.
	cv::Mat mSurroundView = generate_surround_view(mBirdsFront, mBirdsLeft, mBirdsBack, mBirdsRight, nRows, nCols);
	return mSurroundView;
}


bool SurroundView::InitK_G(int nRows, int nCols, float nDx, float nDy){
	cv::Mat mK_G = cv::Mat::zeros(3,3,CV_64FC1);
	mK_G.at<double>(0,0) = 1/nDx;
	mK_G.at<double>(1,1) = -1/nDy;
	mK_G.at<double>(0,2) = nCols/2;
	mK_G.at<double>(1,2) = nRows/2;
	mK_G.at<double>(2,2) =   1.0;

	this->m_mK_G = mK_G;
	return true;
}


bool SurroundView::BindImagePairs(vector<SVPair> gDistortedPairs){
	this->m_gDistortedPairs = gDistortedPairs;
}