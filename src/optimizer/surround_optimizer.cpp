#include "../../include/optimizer/surround_optimizer.h"

using namespace std;

SurroundOptimizer::SurroundOptimizer(	Camera * pFrontCamera, Camera * pLeftCamera,
				 		Camera * pBackCamera,  Camera * pRightCamera)
		: 	m_pFrontCamera(pFrontCamera), m_pLeftCamera(pLeftCamera), 
			m_pBackCamera(pBackCamera), m_pRightCamera(pRightCamera)
{
	//Construct the optimizer.
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> DirectBlock;  // 求解的向量是6＊1的
    typedef g2o::BlockSolverX DirectBlock;
    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ());
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
    //Front and Back are fixed.
    m_pPoseFront->setFixed(true);

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

    //Front and Back are fixed.
    m_pPoseBack->setFixed(true);

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

bool SurroundOptimizer::AddBinaryEdge(	Eigen::Vector3d mPoint3d, int nCameraIndex,
			 			vector<int> gROI, double nMeasurement,
			 			cv::Mat * pGrayImage){

		//Firstly create the point vertex. It's the 3d position of the point.
		g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
		// this->m_gPointVertices.push_back(pPointVertex);
    	pPointVertex->setId(m_nEdgeIndex++);
    	pPointVertex->setEstimate(Eigen::Vector3d(mPoint3d(0 , 0),
                                         		  mPoint3d(1 , 0),
                                          		  mPoint3d(2 , 0)));
    	pPointVertex->setMarginalized(true);
    	pPointVertex->setFixed(true);
    	this->m_iOptimizer.addVertex(pPointVertex);

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

		DirectBinaryEdge* pEdge = new DirectBinaryEdge ();
		pEdge->BindParameters(nFx,
            nFy,
            nCx,
            nCy,
            pGrayImage);

		vector<g2o::VertexSE3Expmap*> gpPoseVertices = {
			m_pPoseFront,
			m_pPoseLeft,
			m_pPoseBack,
			m_pPoseRight
		};
		pEdge->setVertex ( 0, gpPoseVertices[nCameraIndex] );
		pEdge->setVertex ( 1, pPointVertex);
        pEdge->setMeasurement (nMeasurement);
        pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        pEdge->setId ( m_nEdgeIndex++ );
        g2o::RobustKernel * kernel = new g2o::RobustKernelCauchy;
        kernel->setDelta(23.3);
        pEdge->setRobustKernel(kernel);
        m_iOptimizer.addEdge ( pEdge );	
}



bool SurroundOptimizer::AddFixedBinaryEdge(	Eigen::Vector3d mPoint3d, 
								int nCameraIndex1, vector<int> gROI1, double nMeasurement1, cv::Mat * pGrayImage1,
								int nCameraIndex2, vector<int> gROI2, double nMeasurement2, cv::Mat * pGrayImage2){

		//Firstly create the point vertex. It's the 3d position of the point.
		g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
		// this->m_gPointVertices.push_back(pPointVertex);
    	pPointVertex->setId(m_nEdgeIndex++);
    	pPointVertex->setEstimate(Eigen::Vector3d(mPoint3d(0 , 0),
                                         		  mPoint3d(1 , 0),
                                          		  mPoint3d(2 , 0)));
    	pPointVertex->setMarginalized(true);
    	// pPointVertex->setFixed(true);
    	this->m_iOptimizer.addVertex(pPointVertex);

    	//Then create edges.
        vector<Camera *> gpCameras = {
        	m_pFrontCamera,
        	m_pLeftCamera,
        	m_pBackCamera,
        	m_pRightCamera
        };
        //2 cameras.
        Camera * pCamera1 = gpCameras[nCameraIndex1];
        Camera * pCamera2 = gpCameras[nCameraIndex2];

        //Construct the first edge.
        double nFx1, nFy1, nCx1, nCy1;
        //Get the intrinsics.
        nFx1 = pCamera1->m_mK(0 , 0);
        nFy1 = pCamera1->m_mK(1 , 1);
        nCx1 = pCamera1->m_mK(0 , 2) - gROI1[0];
        nCy1 = pCamera1->m_mK(1 , 2) - gROI1[1];

		DirectBinaryEdge* pEdge1 = new DirectBinaryEdge ();
		pEdge1->BindParameters(nFx1,
            nFy1,
            nCx1,
            nCy1,
            pGrayImage1);

        //Construct the second edge.
        double nFx2, nFy2, nCx2, nCy2;
        //Get the intrinsics.
        nFx2 = pCamera2->m_mK(0 , 0);
        nFy2 = pCamera2->m_mK(1 , 1);
        nCx2 = pCamera2->m_mK(0 , 2) - gROI2[0];
        nCy2 = pCamera2->m_mK(1 , 2) - gROI2[1];

		DirectBinaryEdge* pEdge2 = new DirectBinaryEdge ();
		pEdge2->BindParameters(nFx2,
            nFy2,
            nCx2,
            nCy2,
            pGrayImage2);




		//Vertices of camera poses.
		vector<g2o::VertexSE3Expmap*> gpPoseVertices = {
			m_pPoseFront,
			m_pPoseLeft,
			m_pPoseBack,
			m_pPoseRight
		};

		//Robust kernel.
		g2o::RobustKernel * kernel = new g2o::RobustKernelCauchy;
        kernel->setDelta(23.3);

		//Add the first edge.
		pEdge1->setVertex ( 0, gpPoseVertices[nCameraIndex1] );
		pEdge1->setVertex ( 1, pPointVertex);
        pEdge1->setMeasurement (nMeasurement1);
        pEdge1->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        pEdge1->setId ( m_nEdgeIndex++ );
        pEdge1->setRobustKernel(kernel);


        //Add the second edge.
		pEdge2->setVertex ( 0, gpPoseVertices[nCameraIndex2] );
		pEdge2->setVertex ( 1, pPointVertex);
        pEdge2->setMeasurement (nMeasurement2);
        pEdge2->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        pEdge2->setId ( m_nEdgeIndex++ );
        pEdge2->setRobustKernel(kernel);

        m_iOptimizer.addEdge ( pEdge1 );	
        m_iOptimizer.addEdge ( pEdge2 );	
}


bool SurroundOptimizer::AddInverseDepthEdge( 	Eigen::Vector3d mPoint3d,
												int nCameraIndex1,
												int nCameraIndex2,
				 								vector<int> gROI,
				 								double nMeasurement,
				 								cv::Mat * pGrayImage){


    vector<Camera *> gpCameras = {
    	m_pFrontCamera,
    	m_pLeftCamera,
    	m_pBackCamera,
    	m_pRightCamera
    };
    //2 cameras.
    Camera * pCamera1 = gpCameras[nCameraIndex1];
    Camera * pCamera2 = gpCameras[nCameraIndex2];

	Eigen::Vector3d mPoint_Camera1 = pCamera1->m_mT.rotation_matrix() * mPoint3d + 
									 pCamera1->m_mT.translation();

	//Get the normalized coordinate.
	double nInverseDepth = 1/mPoint_Camera1[2];
	double nNormalized_U = mPoint_Camera1[0] * nInverseDepth;
	double nNormalized_V = mPoint_Camera1[1] * nInverseDepth;


	//Firstly create the inverse depth vertex.
	InverseDepthVertex * pInverseDepth = new InverseDepthVertex();
	pInverseDepth->setId(m_nEdgeIndex++);
	pInverseDepth->BindParameters(nNormalized_U,
								  nNormalized_V,
								  pCamera1->m_mT);
	pInverseDepth->setEstimate(nInverseDepth);
	// pInverseDepth->setFixed(true);

	// cout << "Original 3d is " << endl << mPoint3d << endl;
	// cout << "3D point is " << endl << pInverseDepth->Get3DPoint() << endl;

    this->m_iOptimizer.addVertex(pInverseDepth);



    //Construct the edge.
    double nFx, nFy, nCx, nCy;
    //Get the intrinsics.
    nFx = pCamera2->m_mK(0 , 0);
    nFy = pCamera2->m_mK(1 , 1);
    nCx = pCamera2->m_mK(0 , 2) - gROI[0];
    nCy = pCamera2->m_mK(1 , 2) - gROI[1];

    InverseDepthEdge* pEdge = new InverseDepthEdge();
	pEdge->BindParameters(nFx,
        nFy,
        nCx,
        nCy,
        pGrayImage);

	//Vertices of camera poses.
	vector<g2o::VertexSE3Expmap*> gpPoseVertices = {
		m_pPoseFront,
		m_pPoseLeft,
		m_pPoseBack,
		m_pPoseRight
	};

	//Robust kernel.
	g2o::RobustKernel * kernel = new g2o::RobustKernelCauchy;
    kernel->setDelta(23.3);

    //Add the edge.
	pEdge->setVertex ( 0, gpPoseVertices[nCameraIndex2] );
	pEdge->setVertex ( 1, pInverseDepth);
    pEdge->setMeasurement (nMeasurement);
    pEdge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
    pEdge->setId ( m_nEdgeIndex++ );
    pEdge->setRobustKernel(kernel);

    m_iOptimizer.addEdge ( pEdge );	
}




bool SurroundOptimizer::AddEdge(	Eigen::Vector3d mPoint3d, int nCameraIndex,
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

bool SurroundOptimizer::Optimize(){
	cout << "Edge number" << endl << m_iOptimizer.edges().size() << endl;
	m_iOptimizer.initializeOptimization();
    m_iOptimizer.optimize ( 500 );

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