#ifndef FRAME_H_
#define FRAME_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>
#include "../camera/camera.h"
using namespace std;


class Frame
{
public:
	Frame();
	Frame(string aFilename, Camera * pCamera);
	
	//The filename of the image.
	string m_aFilename;
	//The fisheye image.
	cv::Mat m_mFisheyeImage;
	//The camera
	Camera * m_pCamera;
	
};


#endif