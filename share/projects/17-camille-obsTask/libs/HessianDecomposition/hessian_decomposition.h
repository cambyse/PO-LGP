#pragma once

#include <dlib/clustering.h>
#include <Core/array.h>

namespace hessian_decomposition
{
std::vector<unsigned long> spectral_cluster (
    const dlib::matrix<double>& A,
    const unsigned long num_clusters
)
{
    using namespace dlib;

    DLIB_CASSERT(num_clusters > 1,
        "\t std::vector<unsigned long> spectral_cluster(k,samples,num_clusters)"
        << "\n\t num_clusters can't be 0."
        );

    auto K = A; // copy

    matrix<double,0,1> D(K.nr());
    for (long r = 0; r < K.nr(); ++r)
        D(r) = sum(rowm(K,r));
    D = sqrt(reciprocal(D));
    K = diagm(D)*K*diagm(D);
    matrix<double> u,w,v;
    // Use the normal SVD routine unless the matrix is really big, then use the fast
    // approximate version.
    if (K.nr() < 1000)
        svd3(K,u,w,v);
    else
        svd_fast(K,u,w,v, num_clusters+100, 5);
    // Pick out the eigenvectors associated with the largest eigenvalues.
    rsort_columns(v,w);
    v = colm(v, range(0,num_clusters-1));
    // Now build the normalized spectral vectors, one for each input vector.
    std::vector<matrix<double,0,1> > spec_samps, centers;
    for (long r = 0; r < v.nr(); ++r)
    {
        spec_samps.push_back(trans(rowm(v,r)));
        const double len = length(spec_samps.back());
        if (len != 0)
            spec_samps.back() /= len;
    }
    // Finally do the K-means clustering
    pick_initial_centers(num_clusters, centers, spec_samps);
    find_clusters_using_kmeans(spec_samps, centers);
    // And then compute the cluster assignments based on the output of K-means.
    std::vector<unsigned long> assignments;
    for (unsigned long i = 0; i < spec_samps.size(); ++i)
        assignments.push_back(nearest_center(centers, spec_samps[i]));

    return assignments;
}

template< typename T>
void addEdge(uint from, uint to, T & A)
{
  A(from, to) = A(to, from) = 1.0;
};

std::vector<intA> decomposeHessian(const arr& H);
}
