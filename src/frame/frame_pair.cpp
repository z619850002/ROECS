#include "../../include/frame/frame_pair.h"

using namespace std;

//Default constructor
SVPair::SVPair(){

}


SVPair::SVPair(	Frame * pFrontFrame, Frame * pLeftFrame,
				Frame * pBackFrame, Frame * pRightFrame)
	: m_pFrontFrame(pFrontFrame), m_pLeftFrame(pLeftFrame),
	  m_pBackFrame(pBackFrame), m_pRightFrame(pRightFrame)		
{}