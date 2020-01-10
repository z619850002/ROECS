#ifndef FRAME_PAIR_H_
#define FRAME_PAIR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <iostream>
#include <vector>
#include <string>

#include "frame.h"
using namespace std;


//Frame pair of the surround-view image.
class SVPair
{
public:
	SVPair();

	SVPair(	Frame * pFrontFrame, Frame * pLeftFrame,
			Frame * pBackFrame, Frame * pRightFrame);

	Frame * m_pFrontFrame;
	Frame * m_pLeftFrame;
	Frame * m_pBackFrame;
	Frame * m_pRightFrame;
	
};


#endif