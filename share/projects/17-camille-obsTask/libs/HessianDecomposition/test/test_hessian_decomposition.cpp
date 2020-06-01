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
void buildOneLooselyCoupledProblem(T & A)
{
  // cluster 0: 0, 1, 4, 5, 8, 9
  // cluster 1: 2, 3, 6, 7, 10, 11, 12

  CHECK(size(A) >= 13, "please build the matrix correctly");

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
}

arr buildOneLooselyCoupledProblem()
{
  arr H(13, 13);

  buildOneLooselyCoupledProblem(H);

  return H;
}

arr buildTwoIndependantProblems()
{
  arr H(10, 10);

  // cluster 1
  addEdge(0, 1, H);
  addEdge(1, 2, H);
  addEdge(2, 3, H);
  addEdge(3, 4, H);
  addEdge(4, 2, H);

  // cluster 2
  addEdge(5, 6, H);
  addEdge(6, 7, H);
  addEdge(7, 8, H);
  addEdge(8, 9, H);
  addEdge(9, 6, H);

  return H;
}

arr buildOneLooselyCoupledPlusOneIndependantProblem()
{
  arr H(20, 20);

  buildOneLooselyCoupledProblem(H);

  addEdge(13, 14, H);
  addEdge(13, 15, H);
  addEdge(15, 16, H);
  addEdge(14, 16, H);

  // space left in graph, left empty on purpose

  return H;
}

arr buildTwoIndependantProblemsSparse()
{
  auto H = buildTwoIndependantProblems();

  H.special = &H.sparse();

  CHECK(isSparseMatrix(H), "should be a sparse matrix");

  return H;
}

TEST(DLib, UseDLib)
{
  using namespace dlib;

  matrix<double> A(13, 13);

  for(auto i = 0; i < A.nr(); ++i)
    for(auto j = 0; j < A.nc(); ++j)
      A(i, j) = 0.0;

  buildOneLooselyCoupledProblem(A);

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

  buildOneLooselyCoupledProblem(H);

  auto A = buildAdjacancyMatrix(H);
  auto sparsestCut = spectralCluster(A, 2);

  EXPECT_EQ(H.d0, sparsestCut.size());
}

TEST(DecomposeHessian, OneLooselyCoupledProblem)
{
  auto H = buildOneLooselyCoupledProblem();

  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(1, decomp.problems.size());
  EXPECT_EQ(2, decomp.problems.front().xmasks.size());
  EXPECT_EQ(2, decomp.problems.front().sizes.size());
  EXPECT_EQ(7, decomp.problems.front().sizes[0]);
  EXPECT_EQ(8, decomp.problems.front().sizes[1]);
  EXPECT_EQ(1, decomp.problems.front().overlaps[0].size());
  EXPECT_EQ(1, decomp.problems.front().overlaps[1].size());
  EXPECT_EQ(intV({1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0}), decomp.problems.front().xmasks[0]);
  EXPECT_EQ(intV({0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1}), decomp.problems.front().xmasks[1]);
}

TEST(DecomposeHessian, TwoIndependantProblems)
{
  auto H = buildTwoIndependantProblems();

  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(2, decomp.problems.size());
  EXPECT_EQ(intV({1, 1, 1, 1, 1, 0, 0, 0, 0, 0}), decomp.problems[0].xmasks[0]);
  EXPECT_EQ(intV({0, 0, 0, 0, 0, 1, 1, 1, 1, 1}), decomp.problems[1].xmasks[0]);
}

TEST(DecomposeHessian, TwoIndependantProblemsSparse)
{
  auto H = buildTwoIndependantProblemsSparse();

  auto decomp = decomposeSparseHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(2, decomp.problems.size());
  EXPECT_EQ(intV({1, 1, 1, 1, 1, 0, 0, 0, 0, 0}), decomp.problems[0].xmasks[0]);
  EXPECT_EQ(intV({0, 0, 0, 0, 0, 1, 1, 1, 1, 1}), decomp.problems[1].xmasks[0]);
}


TEST(DecomposeHessian, OneLooselyCoupledPlusOneIndependantProblem)
{
  auto H = buildOneLooselyCoupledPlusOneIndependantProblem();
  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(2, decomp.problems.size());
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

