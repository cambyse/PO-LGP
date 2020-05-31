#include <KOMO/komo.h>

#include <hessian_decomposition.h>

#include <gtest/gtest.h>

using namespace std;
using namespace hessian_decomposition;

TEST(DLib, UseDLib)
{
  using namespace dlib;

  matrix<double> A(12, 12);

  for(auto i = 0; i < A.nr(); ++i)
    for(auto j = 0; j < A.nc(); ++j)
      A(i, j) = 0.0;

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

  std::cout << A << std::endl;

  // Finally, we can also solve the same kind of non-linear clustering problem with
  // spectral_cluster().  The output is a vector that indicates which cluster each sample
  // belongs to.  Just like with kkmeans, it assigns each point to the correct cluster.
  std::vector<unsigned long> assignments = spectral_cluster(A, 2);
  cout << mat(assignments) << endl;
}

TEST(DecomposeHessian, ConvertAndDecompose)
{
  arr H;
  auto subs = decomposeHessian(H);
}

////////////////////////////////
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}

