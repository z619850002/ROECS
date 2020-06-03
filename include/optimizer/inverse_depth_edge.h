#ifndef INVERSE_DEPTH_EDGE_H_
#define INVERSE_DEPTH_EDGE_H_

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>



#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>


#include "../camera/camera.h"

using namespace std; 








// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class InverseDepthVertex: public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 	void BindParameters(double nNormalized_U, double nNormalized_V, 
                        Sophus::SE3 mProjectingPose){
 		this->m_nNormalized_U = nNormalized_U;
 		this->m_nNormalized_V = nNormalized_V;
        this->m_mProjectingPose = mProjectingPose;
 	}



 	

    virtual void setToOriginImpl() // 重置
    {
        _estimate = 0;
    }

    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += update[0];
    }

    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}

    //Personal
    //Use the normalized coordinate,
 	//the inverse depth and the projecting pose
 	//to get the 3D point in
 	//the ground coordinate system.
 	Eigen::Vector3d Get3DPoint() const {
 		//Point in the camera coordinate.
 		//Use the inverse depth.
 		//This is in the homogeneous coordinate.
 		Eigen::Vector4d mPoint_Camera = (1/_estimate) * Eigen::Vector4d(
 				this->m_nNormalized_U,
 				this->m_nNormalized_V,
 				1,
 				_estimate
 			);
 		Eigen::Vector4d mHomogeneous = (m_mProjectingPose.inverse()).matrix() * mPoint_Camera;
 		return Eigen::Vector3d(
 				mHomogeneous[0]/mHomogeneous[3],
 				mHomogeneous[1]/mHomogeneous[3],
 				mHomogeneous[2]/mHomogeneous[3]
 			);
 	}

 	//From camera to ground.
 	Eigen::Matrix3d GetRotationToGround() const {
 		return (m_mProjectingPose.inverse()).rotation_matrix();
 	}

 	//Derivatives from the P_i to the inverse depth.
 	Eigen::Vector3d GetDerivatives() const {
 		double nLamda = this->_estimate;
        // Eigen::Vector3d mPoint = m_mProjectingPose.inverse().rotation_matrix() *  Eigen::Vector3d(   this->m_nNormalized_U,
        //                                             this->m_nNormalized_V,
        //                                             1) * (1/nLamda)  + m_mProjectingPose.inverse().translation();
        // cout << "Point3d is " << endl << mPoint << endl;
 		return -(1/(nLamda * nLamda)) * Eigen::Vector3d(
 				this->m_nNormalized_U,
 				this->m_nNormalized_V,
 				1
 			);
 	}




    //Normalized coordinate.
    double m_nNormalized_U;
    double m_nNormalized_V;

    //From ground to camera.
    Sophus::SE3 m_mProjectingPose;
};




//This is a binary edge. The depth of each pixel is considered in the optimization.
class InverseDepthEdge : public g2o::BaseBinaryEdge<1 , double , g2o::VertexSE3Expmap , InverseDepthVertex>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Default Constructor.
        InverseDepthEdge(): g2o::BaseBinaryEdge<1 , double , g2o::VertexSE3Expmap , InverseDepthVertex>() {}
        

        void BindParameters(float nFx , float nFy , 
                        	float nCx , float nCy , cv::Mat * pImage , int nType){
            this->m_pImage = pImage;
            this->m_nFx = nFx;
            this->m_nFy = nFy;
            this->m_nCx = nCx;
            this->m_nCy = nCy;

            this->m_nCameraType = nType;

        }

        InverseDepthVertex * GetDepthVertex(){
            InverseDepthVertex * pPointInverseDepth = static_cast<InverseDepthVertex *>(_vertices[1]);
            return pPointInverseDepth;
        }

        double GetError(){
    //Get the vertex.
            const g2o::VertexSE3Expmap * pPoseVertex = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            //The inverse depth vertex. It can generate the 3d point.
            const InverseDepthVertex * pPointInverseDepth = static_cast<const InverseDepthVertex *>(_vertices[1]);

            //The coordinate in camera coordinate system.
            Eigen::Vector3d mPoint3d = pPointInverseDepth->Get3DPoint();


            Eigen::Vector3d mPoint_Camera = pPoseVertex->estimate().map(mPoint3d);
            //Map the point on the image.
            float nU = mPoint_Camera[0] * m_nFx / mPoint_Camera[2] + m_nCx;
            float nV = mPoint_Camera[1] * m_nFy / mPoint_Camera[2] + m_nCy;

            //Abandon pixels on the boundary of the image.
            if (nU-4 < 0 || nU+4 > m_pImage->cols ||
                nV-4 < 0 || nV+4 > m_pImage->rows ){
                return 0.0;
            }else{
                double nError1 = (this->getPixelValue(nU, nV) - _measurement);
                double nError2 = (this->getPixelValue(nU-4, nV) - _measurement);
                double nError3 = (this->getPixelValue(nU+4, nV) - _measurement);
                double nError4 = (this->getPixelValue(nU, nV-4) - _measurement);
                double nError5 = (this->getPixelValue(nU, nV+4) - _measurement);
                double nError6 = (this->getPixelValue(nU-2, nV-2) - _measurement);
                double nError7 = (this->getPixelValue(nU-2, nV+2) - _measurement);
                double nError8 = (this->getPixelValue(nU+2, nV-2) - _measurement);
                double nError9 = (this->getPixelValue(nU+2, nV+2) - _measurement);

                return (nError1 + nError2  + nError3 + nError4 + nError5 + nError6 + nError7 + nError8 + nError9 )/9.0;
                
                // _error(0 , 0) = this->getPixelValue(nU, nV) - _measurement;
            }
        }

        virtual void computeError(){



            //Get the vertex.
            const g2o::VertexSE3Expmap * pPoseVertex = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);

            // cout << "Camera Index: " << this->m_nCameraType << " , Camera Pose: " << pPoseVertex->estimate() << endl;


            //The inverse depth vertex. It can generate the 3d point.
            const InverseDepthVertex * pPointInverseDepth = static_cast<const InverseDepthVertex *>(_vertices[1]);

            //The coordinate in camera coordinate system.
            Eigen::Vector3d mPoint3d = pPointInverseDepth->Get3DPoint();


            Eigen::Vector3d mPoint_Camera = pPoseVertex->estimate().map(mPoint3d);
            //Map the point on the image.
            float nU = mPoint_Camera[0] * m_nFx / mPoint_Camera[2] + m_nCx;
            float nV = mPoint_Camera[1] * m_nFy / mPoint_Camera[2] + m_nCy;

            //Abandon pixels on the boundary of the image.
            if (nU-4 < 0 || nU+4 > m_pImage->cols ||
                nV-4 < 0 || nV+4 > m_pImage->rows ){
                _error(0 , 0) = 256;
                this->setLevel(1);
            }else{
                double nError1 = (this->getPixelValue(nU, nV) - _measurement);
                double nError2 = (this->getPixelValue(nU-4, nV) - _measurement);
                double nError3 = (this->getPixelValue(nU+4, nV) - _measurement);
                double nError4 = (this->getPixelValue(nU, nV-4) - _measurement);
                double nError5 = (this->getPixelValue(nU, nV+4) - _measurement);
                double nError6 = (this->getPixelValue(nU-2, nV-2) - _measurement);
                double nError7 = (this->getPixelValue(nU-2, nV+2) - _measurement);
                double nError8 = (this->getPixelValue(nU+2, nV-2) - _measurement);
                double nError9 = (this->getPixelValue(nU+2, nV+2) - _measurement);

                // _error(0 , 0) = (nError1 + nError2  + nError3 + nError4 + nError5 + nError6 + nError7 + nError8 + nError9 )/9.0;
                
                _error(0 , 0) = this->getPixelValue(nU, nV) - _measurement;
            }
        }

        virtual void linearizeOplus(){

            if (level() == 1){
                //Out of boundary.
                _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
                _jacobianOplusXj = Eigen::Matrix<double, 1, 1>::Zero();
                return ;
            }
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            _jacobianOplusXj = Eigen::Matrix<double, 1, 1>::Zero();

            //calculate basic attributes.
			//Get the vertex.
            const g2o::VertexSE3Expmap * pPoseVertex = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            //The inverse depth vertex. It can generate the 3d point.
            const InverseDepthVertex * pInverseDepth = static_cast<const InverseDepthVertex *>(_vertices[1]);

            //The coordinate in camera coordinate system.
            Eigen::Vector3d mPoint3d = pInverseDepth->Get3DPoint();

            // cout << "mPoint3d is " << endl << mPoint3d << endl;

         

            Eigen::Vector3d mPoint_Camera = pPoseVertex->estimate().map(mPoint3d);

            //Map the point on the image.
            double nInverseDepth = 1.0 / mPoint_Camera[2];
            double x = mPoint_Camera[0];
            double y = mPoint_Camera[1];
            double z = mPoint_Camera[2];

            float u = mPoint_Camera[0] * m_nFx / mPoint_Camera[2] + m_nCx;
            float v = mPoint_Camera[1] * m_nFy / mPoint_Camera[2] + m_nCy;
 

            //Jacobian.
            Eigen::Matrix<double , 2 , 6> jacobian_uv_ksai;

            jacobian_uv_ksai(0 , 0) = - x*y / (z*z) * m_nFx;
            jacobian_uv_ksai(0 , 1) = (1 + (x*x / (z*z))) * m_nFx;
            jacobian_uv_ksai(0 , 2) = - y/z * m_nFx;
            jacobian_uv_ksai(0 , 3) = m_nFx/z;
            jacobian_uv_ksai(0 , 4) = 0.0;
            jacobian_uv_ksai(0 , 5) = -x/(z*z) * m_nFx;

            jacobian_uv_ksai(1 , 0) = -(1+y*y/(z*z))*m_nFy;
            jacobian_uv_ksai(1 , 1) = x*y / (z*z) * m_nFy;
            jacobian_uv_ksai(1 , 2) = x / z * m_nFy;
            jacobian_uv_ksai(1 , 3) = 0.0;
            jacobian_uv_ksai(1 , 4) = m_nFy / z;
            jacobian_uv_ksai(1 , 5) = -y / (z*z) * m_nFy;

            //1. dI_j/dp_j   2. dp_j/dP_j   3. dP_j/dP_i   4. dP_i/dlamda
            //1. Gradient of I_j at p_j
            //2. About the intrinsic of camera_j
            //3. About the relative pose from camera_i to camera_j
            //4. About the normalized coordinate


            Eigen::Matrix<double , 1 , 2> jacobian_pixel_uv;
            
            jacobian_pixel_uv(0 , 0) = (getPixelValue(u+2, v) - getPixelValue(u-2, v))/4;
            
            jacobian_pixel_uv(0 , 1) = (getPixelValue(u, v+2) - getPixelValue(u, v-2))/4;


            Eigen::Matrix<double , 2 , 3> mJacibian_u_q;
            mJacibian_u_q(0 , 0) = m_nFx/z;
            mJacibian_u_q(0 , 1) = 0.0;
            mJacibian_u_q(0 , 2) =  - m_nFx * x /(z*z);
            mJacibian_u_q(1 , 0) = 0.0;
            mJacibian_u_q(1 , 1) = m_nFy/z;
            mJacibian_u_q(1 , 2) = - m_nFy * y /(z*z);

            Eigen::Matrix3d mRotation = pPoseVertex->estimate().rotation().toRotationMatrix();
            mRotation = mRotation * pInverseDepth->GetRotationToGround();



            _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;


            _jacobianOplusXj = jacobian_pixel_uv * mJacibian_u_q * mRotation * pInverseDepth->GetDerivatives();
        }

        virtual bool read(std::istream& in) {}
        virtual bool write(std::ostream & out) const {}

    private:
        inline float getPixelValue(float x , float y){
            uchar* data = & m_pImage->data[ int ( y ) * m_pImage->step + int ( x ) ];
            float xx = x - floor ( x );
            float yy = y - floor ( y );
            float result = float (
                       ( 1-xx ) * ( 1-yy ) * data[0] +
                       xx* ( 1-yy ) * data[1] +
                       ( 1-xx ) *yy*data[ m_pImage->step ] +
                       xx*yy*data[m_pImage->step+1]
                   );
            return result;
        }

        //The intrinsic of the camera.
        float m_nFx, m_nFy;
        //The center of the image.
        //nCx,nCy is the negative value of the left
        //top coordinate of the undistorted image.
        float m_nCx, m_nCy;
        //Gray image.
        cv::Mat * m_pImage;

        //0-3 corresponding to F-R
        int m_nCameraType;
};



//This is the unary edge, the photometric coefficient is ignored.
//In this edge, the depth of the point is not optimized.
class PriorInvDepthEdge : public g2o::BaseUnaryEdge<1 , double , InverseDepthVertex>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Default Constructor.
        PriorInvDepthEdge() {}

        virtual void computeError(){
            //Get the vertex.
            const InverseDepthVertex * pInvDepthVertex = static_cast<const InverseDepthVertex *>(_vertices[0]);
            //The coordinate in camera coordinate system.
            _error(0 , 0) = pInvDepthVertex->estimate() - _measurement;
            Eigen::Vector3d mPoint3d = pInvDepthVertex->Get3DPoint();
            // cout << "mPoint3d is " << endl << mPoint3d << endl;

        }

        virtual void linearizeOplus(){
            Eigen::Matrix<double , 1 , 1> mJacobian;
            mJacobian(0 , 0) = 1;

            _jacobianOplusXi = mJacobian;
        }

        virtual bool read(std::istream& in) {}
        virtual bool write(std::ostream & out) const {}

};





#endif