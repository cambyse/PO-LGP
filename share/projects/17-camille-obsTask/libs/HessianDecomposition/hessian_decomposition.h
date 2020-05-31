#pragma once

#include <dlib/clustering.h>
#include <Core/array.h>

namespace hessian_decomposition
{
std::vector<unsigned long> spectralCluster (
    const dlib::matrix<double>& A,
    const unsigned long num_clusters
);

dlib::matrix<double> buildAdjacancyMatrix(const arr&);

std::vector<intA> buildDecomposition(const dlib::matrix<double>& A, std::vector<unsigned long> & sparsestCut, uint numberOfCluster);

std::vector<intA> decomposeHessian(const arr& H, uint number_of_cluster);
}
