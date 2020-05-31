#include <KOMO/komo.h>

#include <hessian_decomposition.h>

#include <gtest/gtest.h>

using namespace hessian_decomposition;

uint size(const arr& A)
{
  CHECK_EQ(A.d0, A.d1, "");
  return A.d0;
}

uint size(const dlib::matrix<double> & A)
{
  CHECK_EQ(A.nr(), A.nc(), "");
  return A.nr();
}

template< typename T>
void addEdge(uint from, uint to, T & A)
{
  A(from, to) = A(to, from) = 1.0;
};

template<typename T>
void buildPb1(T & A)
{
  CHECK_EQ(size(A), 13, "please build the matrix correctly");

  addEdge(1, 0, A);
  addEdge(3, 2, A);
  addEdge(4, 0, A);
  addEdge(4, 1, A);
  addEdge(5, 0, A);
  addEdge(5, 1, A);
  addEdge(5, 4, A);
  addEdge(6, 2, A);
  addEdge(6, 3, A);
  addEdge(6, 5, A);
  addEdge(7, 2, A);
  addEdge(7, 3, A);
  addEdge(7, 6, A);
  addEdge(8, 4, A);
  addEdge(8, 5, A);
  addEdge(9, 4, A);
  addEdge(9, 5, A);
  addEdge(9, 8, A);
  addEdge(10, 6, A);
  addEdge(10, 7, A);
  addEdge(11, 6, A);
  addEdge(11, 7, A);
  addEdge(11, 10, A);
  addEdge(11, 12, A);

  // cluster 0: 0, 1, 4, 5, 8, 9
  // cluster 1: 2, 3, 6, 7, 10, 11, 12
}

TEST(DLib, UseDLib)
{
  using namespace dlib;

  matrix<double> A(13, 13);

  for(auto i = 0; i < A.nr(); ++i)
    for(auto j = 0; j < A.nc(); ++j)
      A(i, j) = 0.0;

  buildPb1(A);

  std::cout << A << std::endl;

  // Finally, we can also solve the same kind of non-linear clustering problem with
  // spectral_cluster().  The output is a vector that indicates which cluster each sample
  // belongs to.  Just like with kkmeans, it assigns each point to the correct cluster.,
  std::vector<unsigned long> assignments;
  EXPECT_NO_THROW(assignments = spectralCluster(A, 2));

  std::cout << mat(assignments) << std::endl;
}

TEST(DecomposeHessian, SpectralCluster)
{
  arr H(13, 13);

  buildPb1(H);

  auto A = buildAdjacancyMatrix(H);
  auto sparsestCut = spectralCluster(A, 2);

  EXPECT_EQ(H.d0, sparsestCut.size());
}

TEST(DecomposeHessian, HessianDecomposition)
{
  arr H(13, 13);

  buildPb1(H);

  auto xmasks = decomposeHessian(H, 2);

  EXPECT_EQ(2, xmasks.size());
  EXPECT_EQ(intA(7, {0, 1, 4, 5, 6, 8, 9}), xmasks[0]);
  EXPECT_EQ(intA(8, {2, 3, 6, 5, 7, 10, 11, 12}), xmasks[1]);
}

////////////////////////////////
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}

