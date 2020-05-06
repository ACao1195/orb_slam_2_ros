#ifndef ORBSLAM2_THESIS_H_
#define ORBSLAM2_THESIS_H_

#include <ros/ros.h>
#include "sparse_block_matrix.h"
#include "optimizable_graph.h"
#include "sparse_optimizer.h"

// Initialise global variables to extract covariance matrix
namespace g2o {
	using namespace Eigen;
	extern OptimizableGraph::VertexContainer testVertexContainer; 
	extern SparseBlockMatrix<MatrixXd> testOutputMatrix;
}


#endif