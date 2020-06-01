#pragma once

#include <dlib/clustering.h>
#include <Core/array.h>

namespace hessian_decomposition
{
std::vector<unsigned long> spectralCluster (
    const dlib::matrix<double>& A,
    const unsigned long num_clusters
);

struct Problem
{
  std::vector<intA> xmasks; // vector of LOOSELY coupled subproblems
};

struct Decomposition
{
  std::vector<Problem> problems; // vector of INDEPENDANT subproblems
};

dlib::matrix<double> buildAdjacancyMatrix(const arr&);

Problem buildDecomposition(const dlib::matrix<double>& A, std::vector<unsigned long> & sparsestCut, uint numberOfCluster);

Decomposition decomposeHessian(const arr& H, uint number_of_cluster);
}
