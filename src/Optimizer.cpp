#include<map>
#include<opencv2/core/core.hpp>
#include<g2o/core/g2o_core_api.h>
#include<g2o/core/sparse_optimizer.h>
#include<g2o/types/sba/types_sba.h>
#include<g2o/types/sba/types_six_dof_expmap.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/solvers/eigen/linear_solver_eigen.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>

#include "Optimizer.h"

namespace HIVE_SLAM{

namespace optimizer{

void PointOptimizer::BundleAdjustment(const std::vector<KeyFrame*>& vpKF,
                                      const std::vector<std::vector<PointElement*> >& vvpPoints,
                                      const int nIterations,
                                      bool bRobust)
{
    // Initial solver
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Max keyframe id
    long unsigned int maxKFid = 0;

    // add Keyframe
    for(size_t i = 0; i < vpKF.size(); ++i){
        KeyFrame* pKF = vpKF[i];

        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap(); // Base vertex
        vSE3->setEstimate(pKF->getSE3Pos());

        unsigned long int cur_kf_id = pKF->getId();
        vSE3->setId(cur_kf_id);
        vSE3->setFixed(cur_kf_id==0);

        optimizer.addVertex(vSE3);
        if(cur_kf_id > maxKFid)
            maxKFid=cur_kf_id;

    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // add map points
    std::map<std::string, std::pair<long unsigned int, long unsigned int> > type2id;
    long unsigned int max_id = 0;
    long unsigned int min_id = 0;
    for(size_t i = 0; i < vvpPoints.size(); ++i){
        std::vector<PointElement*> type_pts = vvpPoints[i];
        std::string type = type_pts[0]->getType();
        max_id += type_pts.size();
        type2id[type] = std::pair<long unsigned int, long unsigned int>(min_id, max_id);

        for(size_t j = 0; j < type_pts.size(); ++j){
            PointElement* pMP = type_pts[i];

            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();

            vPoint->setEstimate(pMP->getVectorPos());

            const int id = pMP->getId() + min_id + maxKFid + 1;
            vPoint->setEstimate(id);

            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const std::map<KeyFrame*, int> observations = pMP->getObservations();

            size_t nEdges = 0;

            // add edge
            for(std::map<KeyFrame*, int>::const_iterator it = observations.begin(); it != observations.end(); ++it){
                KeyFrame* pKF = it->first;

                if(pKF->getId() > maxKFid)
                    continue;
                nEdges += 1;

                PointElement* Point = pKF->getPoint(type, it->second);
                cv::Mat obs_ = pKF->ProjectPoint(Point);
                Eigen::Matrix<double,2,1> obs;
                obs << obs_.at<double>(0, 0), obs_.at<double>(1, 0);

                // add edge
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getId())));

                e->setMeasurement(obs);
                e->setInformation(Eigen::Matrix2d::Identity());

                if(bRobust){
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                // add inner parameters
                const cv::Mat K = pKF->getInner();
                e->fx = K.at<double>(0, 0);
                e->fy = K.at<double>(1, 0);
                e->cx = K.at<double>(0, 2);
                e->cy = K.at<double>(1, 2);

                optimizer.addEdge(e);

            }
            
        }

        min_id += type_pts.size();
    }
    // start optimize
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

}

};

};