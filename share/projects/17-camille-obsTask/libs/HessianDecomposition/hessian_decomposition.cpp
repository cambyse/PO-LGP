#include <hessian_decomposition.h>
#include <unordered_map>

namespace hessian_decomposition
{

std::vector<unsigned long> spectralCluster (
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

dlib::matrix<double> buildAdjacancyMatrix(const arr& H)
{
  dlib::matrix<double> A(H.d0, H.d1);

  // hessian
  if(isSparseMatrix(H))
  {
    for(auto i = 0; i < H.d0; ++i)
    {
      for(auto j = 0; j < H.d1; ++j)
      {
        A(i, j) = 0;
      }
    }

    auto Hs = dynamic_cast<rai::SparseMatrix*>(H.special);

    const auto & nzs = Hs->elems;

    for(auto i = 0; i < nzs.d0; ++i)
    {
      auto I = nzs(i, 0);
      auto J = nzs(i, 1);

      A(I, J) = 1;
    }
    //      for(uint i=0;i<Hs->d0;i++)
    //      {
    //        A = Hs->elem(i, i);
    //      }
  }
  else
  {
    for(auto i = 0; i < H.d0; ++i)
    {
      for(auto j = 0; j < H.d1; ++j)
      {
        const auto v = H(i, j);
        if(fabs(v)>1e-7)
          A(i, j) = H(i, j);
        else
          A(i, j) = 0;
      }
    }
  }

  return A;
}

Problem buildDecomposition(const dlib::matrix<double>& A, std::vector<unsigned long> & sparsestCut, uint numberOfCluster)
{
  Problem pb;
  pb.xmasks = std::vector<intA>(numberOfCluster);
  for(auto & xmask: pb.xmasks)
    xmask.reserve(1.2 * sparsestCut.size() / (numberOfCluster)); // account for cluster unbalancing

  for(auto i = 0; i < sparsestCut.size(); ++i)
  {
    const auto & k = sparsestCut[i];
    pb.xmasks[k].append(i);

    for(auto j = 0; j < A.nc(); ++j)
    {
      if(A(i, j) != 0 && sparsestCut[j] != k) // add the neighbors in other cut! crucial part for ADMM
      {
        pb.xmasks[k].append(j);
      }
    }
  }

  return pb;
}

Decomposition decomposeHessian(const arr& H, uint numberOfCluster)
{
  CHECK_EQ(H.d0, H.d1, "hessian should be a square matrix");

  auto A = buildAdjacancyMatrix(H);

  std::cout << A << std::endl;

  auto sparsestCut = spectralCluster(A, numberOfCluster);

  auto xmasks = buildDecomposition(A, sparsestCut, numberOfCluster);

  Decomposition decomp;
  decomp.problems.push_back(xmasks);

  return decomp;
}
}
