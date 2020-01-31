#ifndef SURROUND_OPTIMIZER_H_
#define SURROUND_OPTIMIZER_H_

#include "direct_unary_edge.h"
#include "../camera/camera.h"

using namespace std;

class SurroundOptimizer
{
public:
	//Constructor.
	SurroundOptimizer(	Camera * pFrontCamera, Camera * pLeftCamera,
				 		Camera * pBackCamera,  Camera * pRightCamera)
		: 	m_pFrontCamera(pFrontCamera), m_pLeftCamera(pLeftCamera), 
			m_pBackCamera(pBackCamera), m_pRightCamera(pRightCamera)
	{
		//Construct the optimizer.
	    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
	    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
	    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
	    std::unique_ptr<DirectBlock> pSolverPtr (new DirectBlock ( std::move(pLinearSolver) ));
	    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
	    g2o::OptimizationAlgorithmLevenberg* pSolver = new g2o::OptimizationAlgorithmLevenberg ( std::move(pSolverPtr) ); // L-M
		//TODO: These configuration should be adjusted in the future.    
	    m_iOptimizer.setAlgorithm ( pSolver );
	    m_iOptimizer.setVerbose( true );

	    //Add pose vertices.
	    //Add front camera pose.
	    m_pPoseFront = new g2o::VertexSE3Expmap();
	    m_pPoseFront->setEstimate ( g2o::SE3Quat ( 	m_pFrontCamera->m_mT.rotation_matrix() ,
	    								   			m_pFrontCamera->m_mT.translation() ));
	    //Front ID is 0.
	    m_pPoseFront->setId ( 0 );
	    m_iOptimizer.addVertex ( m_pPoseFront );


	    //Add left camera.
	    m_pPoseLeft = new g2o::VertexSE3Expmap();
	    m_pPoseLeft->setEstimate ( g2o::SE3Quat ( 	m_pLeftCamera->m_mT.rotation_matrix() ,
	    								   			m_pLeftCamera->m_mT.translation() ));

	    //Left ID is 1.
	    m_pPoseLeft->setId ( 1 );
	    m_iOptimizer.addVertex ( m_pPoseLeft );


	    //Add back camera.
	    m_pPoseBack = new g2o::VertexSE3Expmap();
	    m_pPoseBack->setEstimate ( g2o::SE3Quat ( 	m_pBackCamera->m_mT.rotation_matrix() ,
	    								   			m_pBackCamera->m_mT.translation() ));

	    //Back ID is 2.
	    m_pPoseBack->setId ( 2 );
	    m_iOptimizer.addVertex ( m_pPoseBack );


	    //Add back camera.
	    m_pPoseRight = new g2o::VertexSE3Expmap();
	    m_pPoseRight->setEstimate ( g2o::SE3Quat ( 	m_pRightCamera->m_mT.rotation_matrix() ,
	    								   			m_pRightCamera->m_mT.translation() ));

	    //Right ID is 3.
	    m_pPoseRight->setId ( 3 );
	    m_iOptimizer.addVertex ( m_pPoseRight );

	    m_nEdgeIndex = 4;
	}

	bool AddEdge(	Eigen::Vector3d mPoint3d, int nCameraIndex,
				 	vector<int> gROI, double nMeasurement,
				 	cv::Mat * pGrayImage){

            double nFx, nFy, nCx, nCy;
            vector<Camera *> gpCameras = {
            	m_pFrontCamera,
            	m_pLeftCamera,
            	m_pBackCamera,
            	m_pRightCamera
            };
            Camera * pCamera = gpCameras[nCameraIndex];

            //Get the intrinsics.
            nFx = pCamera->m_mK(0 , 0);
            nFy = pCamera->m_mK(1 , 1);
            nCx = pCamera->m_mK(0 , 2) - gROI[0];
            nCy = pCamera->m_mK(1 , 2) - gROI[1];

			DirectUnaryEdge* pEdge = new DirectUnaryEdge (
	            mPoint3d,
	            nFx,
	            nFy,
	            nCx,
	            nCy,
	            pGrayImage
	        );

			vector<g2o::VertexSE3Expmap*> gpPoseVertices = {
				m_pPoseFront,
				m_pPoseLeft,
				m_pPoseBack,
				m_pPoseRight
			};
	        pEdge->setVertex ( 0, gpPoseVertices[nCameraIndex] );
	        pEdge->setMeasurement (nMeasurement);
	        pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
	        pEdge->setId ( m_nEdgeIndex++ );
	        m_iOptimizer.addEdge ( pEdge );	
	}

	bool Optimize(){
		cout << "Edge number" << endl << m_iOptimizer.edges().size() << endl;
		m_iOptimizer.initializeOptimization();
	    m_iOptimizer.optimize ( 100 );

	    //Load poses.
	    //Front.
	    Eigen::Isometry3d mTcw_Front = m_pPoseFront->estimate();
	    m_pFrontCamera->m_mT = Sophus::SE3(mTcw_Front.rotation() , mTcw_Front.translation());

	    //Left
	    Eigen::Isometry3d mTcw_Left = m_pPoseLeft->estimate();
	    m_pLeftCamera->m_mT = Sophus::SE3(mTcw_Left.rotation() , mTcw_Left.translation());

	    //Back.
	    Eigen::Isometry3d mTcw_Back = m_pPoseBack->estimate();
	    m_pBackCamera->m_mT = Sophus::SE3(mTcw_Back.rotation() , mTcw_Back.translation());

	    //Right.
	    Eigen::Isometry3d mTcw_Right = m_pPoseRight->estimate();
	    m_pRightCamera->m_mT = Sophus::SE3(mTcw_Right.rotation() , mTcw_Right.translation());
	}

	//The optimizer.
	g2o::SparseOptimizer m_iOptimizer;

	//Pose vertex.
	g2o::VertexSE3Expmap* m_pPoseFront;
	g2o::VertexSE3Expmap* m_pPoseLeft;
	g2o::VertexSE3Expmap* m_pPoseBack;
	g2o::VertexSE3Expmap* m_pPoseRight;


	//Cameras.
	Camera * m_pFrontCamera;
	Camera * m_pLeftCamera;
	Camera * m_pBackCamera;
	Camera * m_pRightCamera;

	int m_nEdgeIndex;
	
};

#endif