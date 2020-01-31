#ifndef DIRECT_UNARY_EDGE_H_
#define DIRECT_UNARY_EDGE_H_

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
using namespace std; 




//This is the unary edge, the photometric coefficient is ignored.
//In this edge, the depth of the point is not optimized.
class DirectUnaryEdge : public g2o::BaseUnaryEdge<1 , double , g2o::VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Default Constructor.
        DirectUnaryEdge() {}
        //Constructor.
        DirectUnaryEdge(Eigen::Vector3d mPoint3d , float nFx , float nFy , 
                        float nCx , float nCy , cv::Mat * pImage):
                        m_mPoint3d(mPoint3d), m_nFx(nFx), m_nFy(nFy),
                        m_nCx(nCx), m_nCy(nCy), m_pImage(pImage){
                        }

        virtual void computeError(){
            //Get the vertex.
            const g2o::VertexSE3Expmap * pPoseVertex = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            //The coordinate in camera coordinate system.
            Eigen::Vector3d mPoint_Camera = pPoseVertex->estimate().map(m_mPoint3d);
            //Map the point on the image.
            float nU = mPoint_Camera[0] * m_nFx / mPoint_Camera[2] + m_nCx;
            float nV = mPoint_Camera[1] * m_nFy / mPoint_Camera[2] + m_nCy;
            //Abandon pixels on the boundary of the image.
            if (nU-4 < 0 || nU+4 > m_pImage->cols ||
                nV-4 < 0 || nV+4 > m_pImage->rows){
                _error(0 , 0) = 0.0;
                this->setLevel(1);
            }else{
                _error(0 , 0) = this->getPixelValue(nU, nV) - _measurement;
            }
        }

        virtual void linearizeOplus(){
            if (level() == 1){
                //Out of boundary.
                _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            }
            //calculate basic attributes.
            const g2o::VertexSE3Expmap * pPoseVertex = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            //The coordinate in camera coordinate system.
            Eigen::Vector3d mPoint_Camera = pPoseVertex->estimate().map(m_mPoint3d);
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

            Eigen::Matrix<double , 1 , 2> jacobian_pixel_uv;
            jacobian_pixel_uv(0 , 0) = (getPixelValue(u+1, v) - getPixelValue(u-1, v))/2;

            jacobian_pixel_uv(0 , 1) = (getPixelValue(u, v+1) - getPixelValue(u, v-1))/2;

            _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;
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


        Eigen::Vector3d m_mPoint3d;
        //The intrinsic of the camera.
        float m_nFx = 0, m_nFy = 0;
        //The center of the image.
        //nCx,nCy is the negative value of the left
        //top coordinate of the undistorted image.
        float m_nCx = 0, m_nCy = 0;
        //Gray image.
        cv::Mat * m_pImage = nullptr;
};



#endif