#ifndef ORBSLAM2_THESIS_H_
#define ORBSLAM2_THESIS_H_

#include <ros/ros.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_optimizer.h>

// Initialise global variables to extract covariance matrix
namespace g2o {
	using namespace Eigen;
	extern OptimizableGraph::VertexContainer testVertexContainer; 
	extern SparseBlockMatrix<MatrixXd> testOutputMatrix;
}


#endif