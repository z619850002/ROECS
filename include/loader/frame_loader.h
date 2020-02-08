#ifndef FRAME_LOADER_H_
#define FRAME_LOADER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>
#include <string>

#include "../frame/frame_pair.h"
#include "../camera/camera.h"
using namespace std;


class FrameLoader
{
public:
	FrameLoader();

	FrameLoader(string aDirectoryName , Camera * pFrontCamera,
				Camera * pLeftCamera,  Camera * pBackCamera, 
				Camera * pRightCamera);

	bool LoadFilenames(string aDirectoryName);

	vector<SVPair> LoadFramePairs(vector<int> gIndexes); 

	vector<SVPair> LoadAll(); 

	//The directory of all data.
	string m_aDirectoryName;
	//The filenames of frames captured by different cameras.
	vector<string> m_gFrontFilenames;
	vector<string> m_gLeftFilenames;
	vector<string> m_gBackFilenames;
	vector<string> m_gRightFilenames;

	//Cameras.
	Camera * m_pFrontCamera;
	Camera * m_pLeftCamera;
	Camera * m_pBackCamera;
	Camera * m_pRightCamera;
	
};


#endif