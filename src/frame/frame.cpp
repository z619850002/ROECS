#include "../../include/frame/frame.h"

using namespace std;

Frame::Frame(){

}


Frame::Frame(string aFilename, Camera * pCamera)
	: m_aFilename(aFilename), m_pCamera(pCamera)
{
	this->m_mFisheyeImage = cv::imread(aFilename);
}