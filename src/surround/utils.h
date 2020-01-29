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



cv::Mat ground2cam(int x,int y, cv::Mat K_G, Sophus::SE3 T_CG, Eigen::Matrix3d K_C)
{
	cv::Mat p_G = cv::Mat::ones(3,1,CV_64FC1);
	p_G.at<double>(0,0) = x;
	p_G.at<double>(1,0) = y;
// 	cout<<p_G<<endl;
	cv::Mat P_G = cv::Mat::ones(4,1,CV_64FC1);
	P_G(cv::Rect(0,0,1,3)) = K_G.inv()*p_G;
	P_G.at<double>(0,2) = 0;
// 	cout<<P_G<<endl;
	cv::Mat P_C = eigen2mat(T_CG.matrix())*P_G;
// 	cout<<P_C<<endl;
	cv::Mat P_C_1 = P_C(cv::Rect(0,0,1,3))/P_C.at<double>(0,2);
// 	cout<<P_C_1<<endl;
	cv::Mat p_C = eigen2mat(K_C)*P_C_1;
// 	cout<<p_C<<endl;
	
	return p_C;
}


//Get the undistorted ROI on one image.
void CalculateROI(	cv::Mat & mImage , cv::Mat & mROI , 
					cv::Mat mK , cv::Mat mD ,
					Sophus::SE3 sT_CG , cv::Mat mKG,
					int nROI_X , int nROI_Y , int nROI_W , int nROI_H,
					vector<int> & gROI){
	//The inverse pose of the camera.
	Sophus::SE3 sT_GC = sT_CG.inverse();

	Eigen::Matrix3d mEigenK;
	cv2eigen(mK , mEigenK);

	cv::Mat mMinUPoint = ground2cam(nROI_X,    nROI_Y,
								mKG, sT_CG, mEigenK);
	cv::Mat mMaxUPoint = ground2cam(nROI_X + nROI_W,    nROI_Y,
								mKG, sT_CG, mEigenK);
	cv::Mat mMinVPoint = ground2cam(nROI_X,    nROI_Y + nROI_H,
								mKG, sT_CG, mEigenK);
	cv::Mat mMaxVPoint = ground2cam(nROI_X + nROI_W,    nROI_Y + nROI_H,
								mKG, sT_CG, mEigenK);

	
// 	cout<<"--------------------determin ROI on camera A--------------------------"<<endl;
	vector<double> gPointsX{
						mMinUPoint.at<double>(0,0),
						mMaxUPoint.at<double>(0,0),
						mMinVPoint.at<double>(0,0),
						mMaxVPoint.at<double>(0,0)
					};

	vector<double> gPointsY{
						mMinUPoint.at<double>(1,0),
						mMaxUPoint.at<double>(1,0),
						mMinVPoint.at<double>(1,0),
						mMaxVPoint.at<double>(1,0)
					};

	//Get the whole area of the ROI
	int nMaxX = int(*max_element(gPointsX.begin(),gPointsX.end()))+10;
	int nMinX = int(*min_element(gPointsX.begin(),gPointsX.end()))-10;
	int nMaxY = int(*max_element(gPointsY.begin(),gPointsY.end()))+10;
	int nMinY = int(*min_element(gPointsY.begin(),gPointsY.end()))-10;
	int nWidth  = nMaxX - nMinX;
	int nHeight = nMaxY - nMinY;

	//Save ROI region.
	gROI.clear();
	gROI.reserve(4);
	gROI.push_back(nMinX);
	gROI.push_back(nMinY);
	gROI.push_back(nMaxX);
	gROI.push_back(nMaxY);


	cv::Rect iROI(nMinX, nMinY, nWidth, nHeight);
	
	cv::Size iSizeROI(nWidth,nHeight);

	//Generate new ROI K.
	cv::Mat mK_ROI = mK.clone();
	mK_ROI.at<double>(0,2) = mK_ROI.at<double>(0,2) - nMinX;
	mK_ROI.at<double>(1,2) = mK_ROI.at<double>(1,2) - nMinY;
	//Undistort image.
	cv::fisheye::undistortImage(mImage,mROI,
								mK,mD,
								mK_ROI,iSizeROI);


}
