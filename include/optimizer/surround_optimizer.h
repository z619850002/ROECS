#ifndef SURROUND_OPTIMIZER_H_
#define SURROUND_OPTIMIZER_H_

#include "direct_unary_edge.h"
#include "direct_binary_edge.h"
#include "inverse_depth_edge.h"
#include "../camera/camera.h"

#include <g2o/solvers/eigen/linear_solver_eigen.h>
//Robust kernel
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "../utils/utils.h"
using namespace std;

class SurroundOptimizer
{
public:
	//Constructor.
	SurroundOptimizer(	Camera * pFrontCamera, Camera * pLeftCamera,
				 		Camera * pBackCamera,  Camera * pRightCamera);

	bool AddBinaryEdge(	Eigen::Vector3d mPoint3d, int nCameraIndex,
				 		vector<int> gROI, double nMeasurement,
				 		cv::Mat * pGrayImage);

	bool AddFixedBinaryEdge(	Eigen::Vector3d mPoint3d, 
								int nCameraIndex1, vector<int> gROI1, double nMeasurement1, cv::Mat * pGrayImage1,
								int nCameraIndex2, vector<int> gROI2, double nMeasurement2, cv::Mat * pGrayImage2);

	bool AddEdge(	Eigen::Vector3d mPoint3d, int nCameraIndex,
				 	vector<int> gROI, double nMeasurement,
				 	cv::Mat * pGrayImage);


	//CameraIndex1 is used to generate normalized coordinate.
	bool AddInverseDepthEdge( 	Eigen::Vector3d mPoint3d,
								int nCameraIndex1,
								int nCameraIndex2,
				 				vector<int> gROI, double nMeasurement,
				 				cv::Mat * pGrayImage);

	bool Optimize();


	//The optimizer.
	g2o::SparseOptimizer m_iOptimizer;

	//Pose vertex.
	g2o::VertexSE3Expmap* m_pPoseFront;
	g2o::VertexSE3Expmap* m_pPoseLeft;
	g2o::VertexSE3Expmap* m_pPoseBack;
	g2o::VertexSE3Expmap* m_pPoseRight;


	vector<g2o::VertexSBAPointXYZ *> m_gPointVertices;

	//Cameras.
	Camera * m_pFrontCamera;
	Camera * m_pLeftCamera;
	Camera * m_pBackCamera;
	Camera * m_pRightCamera;

	int m_nEdgeIndex;
	
};




#endif