#include "../../include/loader/frame_loader.h"

using namespace std;

//Default constructor.
FrameLoader::FrameLoader(){

}

FrameLoader::FrameLoader(string aDirectoryName , Camera * pFrontCamera,
						Camera * pLeftCamera,  Camera * pBackCamera, 
						Camera * pRightCamera)
	: 	m_aDirectoryName(aDirectoryName), m_pFrontCamera(pFrontCamera),
		m_pLeftCamera(pLeftCamera), m_pBackCamera(pBackCamera),
		m_pRightCamera(pRightCamera)
{

}

bool FrameLoader::LoadFilenames(string aDirectoryName){
	//Load the filenames.
	vector<cv::String> gAllFileNames;
	cv::glob(aDirectoryName, gAllFileNames);

	//Not valid directory.
	if (gAllFileNames.size()%4 != 0){
		cout << "Wrong number of images" << endl;
		return false;
	}

	// for (auto item : gAllFileNames){
	// 	cout << item << endl;
	// }

	for (int i=0;i<gAllFileNames.size()/4;i++){
		this->m_gBackFilenames.push_back((string)gAllFileNames[i*4]);
		this->m_gFrontFilenames.push_back((string)gAllFileNames[i*4+1]);
		this->m_gLeftFilenames.push_back((string)gAllFileNames[i*4+2]);
		this->m_gRightFilenames.push_back((string)gAllFileNames[i*4+3]);
	}

	return true;
}

vector<SVPair> FrameLoader::LoadFramePairs(vector<int> gIndexes){
	LoadFilenames(this->m_aDirectoryName);
	vector<SVPair> gPairs;
	for (auto nIndex : gIndexes){
		//Get the Frame.
		Frame * pFrontFrame = new Frame(m_gFrontFilenames[nIndex] , m_pFrontCamera);
		Frame * pLeftFrame = new Frame(m_gLeftFilenames[nIndex] , m_pLeftCamera);
		Frame * pBackFrame = new Frame(m_gBackFilenames[nIndex] , m_pBackCamera);
		Frame * pRightFrame = new Frame(m_gRightFilenames[nIndex] , m_pRightCamera);
		//Construct the pair.
		gPairs.push_back(SVPair(pFrontFrame, pLeftFrame, pBackFrame, pRightFrame));
	}
	return gPairs;
}


vector<SVPair> FrameLoader::LoadAll(){
	LoadFilenames(this->m_aDirectoryName);
	vector<SVPair> gPairs;
	for (int nIndex = 0; nIndex < this->m_gFrontFilenames.size(); nIndex++){
		//Get the Frame.
		Frame * pFrontFrame = new Frame(m_gFrontFilenames[nIndex] , m_pFrontCamera);
		Frame * pLeftFrame = new Frame(m_gLeftFilenames[nIndex] , m_pLeftCamera);
		Frame * pBackFrame = new Frame(m_gBackFilenames[nIndex] , m_pBackCamera);
		Frame * pRightFrame = new Frame(m_gRightFilenames[nIndex] , m_pRightCamera);
		//Construct the pair.
		gPairs.push_back(SVPair(pFrontFrame, pLeftFrame, pBackFrame, pRightFrame));
	}
	return gPairs;
}