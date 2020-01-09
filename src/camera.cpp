#include "../include/camera/camera.h"

//Constructor.
Camera::Camera(){
	this->m_mR = (cv::Mat_<double> (3,3) << 	1.0 , 0.0 , 0.0,
											0.0 , 1.0 , 0.0,
											0.0 , 0.0 , 1.0);
	this->m_mt = (cv::Mat_<double> (3,1) << 0.0 , 0.0 , 0.0);
}



//Setter.
void Camera::SetName(string aCameraName){
	this->m_aCameraName = aCameraName;
}

void Camera::SetR(cv::Mat mMat){
	this->m_mR = mMat;
}

void Camera::SetT(cv::Mat mVec){
	this->m_mt = mVec;
}

void Camera::SetK(cv::Mat mK){
	this->m_mK = mK;
}

void Camera::SetD(cv::Mat mD){
	this->m_mD = mD;
}


//Getter.
string Camera::GetName(){
	return this->m_aCameraName;
}

cv::Mat Camera::GetIsometry(){
	cv::Mat mIsometry;
	cv::Mat mBottom = (cv::Mat_<double>(1 , 4) << 0 , 0 , 0 , 1);
	cv::hconcat(this->m_mR , this->m_mt, mIsometry);
	cv::vconcat(mIsometry, mBottom , mIsometry);
	return mIsometry;
}

cv::Mat Camera::GetRotation(){
	return this->m_mR;
}

cv::Mat Camera::GetTranslation(){
	return this->m_mt;
}

cv::Mat Camera::GetK(){
	return this->m_mK;
}

cv::Mat Camera::GetD(){
	return this->m_mD;
}
