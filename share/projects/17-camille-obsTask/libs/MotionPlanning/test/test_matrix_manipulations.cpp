#include "komo_tree.h"
#include <gtest/gtest.h>
#include <functional>

//#include <eigen3/Eigen/SparseCholesky>

#include <Core/graph.h>
#include <Core/array.h>

#include <tree_builder.h>

using namespace std;
using namespace mp;

/*
 *
 * Tests for testing various linear algebra things
 *
 */

TEST(RowShifting, Understanding_RowShifting)
{
  arr J;
  uint xN = 20;
  uint band_size = 6;// (k+1)*dim_xmax
  uint phiN = xN * 3; // xN * nTask * taskDim
  RowShifted *Jaux = makeRowShifted(J, phiN, band_size, xN);

  EXPECT_EQ(J.d0, phiN);
  EXPECT_EQ(J.d1, band_size);
  EXPECT_EQ(Jaux->real_d1, xN);

  // assign
  J(0, 0) = 1; // store phiN * band_size elements
  J(1, Jaux->rowShift(1)) = 1;

  //J(30, 1) = 1;

  // necessary?
  Jaux->reshift();
  Jaux->computeColPatches(true);

  // unshift
  auto Junshifted = unpack(J); // store phiN * xN

  EXPECT_EQ(Junshifted.d0, phiN);
  EXPECT_EQ(Junshifted.d1, xN);
  EXPECT_EQ(Junshifted(0, 0), 1);
  //EXPECT_EQ(Junshifted(1, 1), 1);
}

//std::vector< Eigen::Triplet< double > > get_a_values()
//{
//  return {{0,0,48400},{0,2,-32200},{0,4,8000},{1,1,48400},{1,3,-32200},{1,5,8000},{2,0,-32200},{2,2,48400},{2,4,-32200},{2,6,8000},{3,1,-32200},{3,3,48400},{3,5,-32200},{3,7,8000},{4,0,8000},{4,2,-32200},{4,4,48400},{4,6,-32200},{4,8,8000},{5,1,8000},{5,3,-32200},{5,5,48400},{5,7,-32200},{5,9,8000},{6,2,8000},{6,4,-32200},{6,6,48400},{6,8,-32200},{6,10,8000},{7,3,8000},{7,5,-32200},{7,7,48400},{7,9,-32200},{7,11,8000},{8,4,8000},{8,6,-32200},{8,8,48400},{8,10,-32200},{8,12,8000},{9,5,8000},{9,7,-32200},{9,9,48400},{9,11,-32200},{9,13,8000},{10,6,8000},{10,8,-32200},{10,10,48400},{10,12,-32200},{10,14,8000},{11,7,8000},{11,9,-32200},{11,11,48400},{11,13,-32200},{11,15,8000},{12,8,8000},{12,10,-32200},{12,12,48400},{12,14,-32200},{12,16,8000},{13,9,8000},{13,11,-32200},{13,13,48400},{13,15,-32200},{13,17,8000},{14,10,8000},{14,12,-32200},{14,14,48400},{14,16,-32200},{14,18,8000},{15,11,8000},{15,13,-32200},{15,15,48400},{15,17,-32200},{15,19,8000},{16,12,8000},{16,14,-32200},{16,16,48400},{16,18,-32200},{16,20,8000},{17,13,8000},{17,15,-32200},{17,17,48400},{17,19,-32200},{17,21,8000},{18,14,8000},{18,16,-32200},{18,18,48400},{18,20,-32200},{18,22,8000},{19,15,8000},{19,17,-32200},{19,19,48400},{19,21,-32200},{19,23,8000},{20,16,8000},{20,18,-32200},{20,20,48400},{20,22,-32200},{20,24,8000},{21,17,8000},{21,19,-32200},{21,21,48400},{21,23,-32200},{21,25,8000},{22,18,8000},{22,20,-32200},{22,22,48400},{22,24,-32200},{22,26,8000},{23,19,8000},{23,21,-32200},{23,23,48400},{23,25,-32200},{23,27,8000},{24,20,8000},{24,22,-32200},{24,24,48400},{24,26,-32200},{24,28,8000},{25,21,8000},{25,23,-32200},{25,25,48400},{25,27,-32200},{25,29,8000},{26,22,8000},{26,24,-32200},{26,26,48400},{26,28,-32200},{26,30,8000},{27,23,8000},{27,25,-32200},{27,27,48400},{27,29,-32200},{27,31,8000},{28,24,8000},{28,26,-32200},{28,28,48400},{28,30,-32200},{28,32,8000},{29,25,8000},{29,27,-32200},{29,29,48400},{29,31,-32200},{29,33,8000},{30,26,8000},{30,28,-32200},{30,30,48400},{30,32,-32200},{30,34,8000},{31,27,8000},{31,29,-32200},{31,31,48400},{31,33,-32200},{31,35,8000},{32,28,8000},{32,30,-32200},{32,32,48400},{32,34,-32200},{32,36,4000},{32,76,4000},{33,29,8000},{33,31,-32200},{33,33,48400},{33,35,-32200},{33,37,4000},{33,77,4000},{34,30,8000},{34,32,-32200},{34,34,48400},{34,36,-16200},{34,38,4000},{34,76,-16000},{34,78,4000},{35,31,8000},{35,33,-32200},{35,35,48400},{35,37,-16200},{35,39,4000},{35,77,-16000},{35,79,4000},{36,32,4000},{36,34,-16200},{36,36,24400},{36,38,-16200},{36,40,4000},{37,33,4000},{37,35,-16200},{37,37,24400},{37,39,-16200},{37,41,4000},{38,34,4000},{38,36,-16200},{38,38,24400},{38,40,-16200},{38,42,4000},{39,35,4000},{39,37,-16200},{39,39,24400},{39,41,-16200},{39,43,4000},{40,36,4000},{40,38,-16200},{40,40,24400},{40,42,-16200},{40,44,4000},{41,37,4000},{41,39,-16200},{41,41,24400},{41,43,-16200},{41,45,4000},{42,38,4000},{42,40,-16200},{42,42,24400},{42,44,-16200},{42,46,4000},{43,39,4000},{43,41,-16200},{43,43,24400},{43,45,-16200},{43,47,4000},{44,40,4000},{44,42,-16200},{44,44,24400},{44,46,-16200},{44,48,4000},{45,41,4000},{45,43,-16200},{45,45,24400},{45,47,-16200},{45,49,4000},{46,42,4000},{46,44,-16200},{46,46,24400},{46,48,-16200},{46,50,4000},{47,43,4000},{47,45,-16200},{47,47,24400},{47,49,-16200},{47,51,4000},{48,44,4000},{48,46,-16200},{48,48,24400},{48,50,-16200},{48,52,4000},{49,45,4000},{49,47,-16200},{49,49,24400},{49,51,-16200},{49,53,4000},{50,46,4000},{50,48,-16200},{50,50,24400},{50,52,-16200},{50,54,4000},{51,47,4000},{51,49,-16200},{51,51,24400},{51,53,-16200},{51,55,4000},{52,48,4000},{52,50,-16200},{52,52,24400},{52,54,-16200},{52,56,4000},{53,49,4000},{53,51,-16200},{53,53,24400},{53,55,-16200},{53,57,4000},{54,50,4000},{54,52,-16200},{54,54,24400},{54,56,-16200},{54,58,4000},{55,51,4000},{55,53,-16200},{55,55,24400},{55,57,-16200},{55,59,4000},{56,52,4000},{56,54,-16200},{56,56,24400},{56,58,-16200},{56,60,4000},{57,53,4000},{57,55,-16200},{57,57,24400},{57,59,-16200},{57,61,4000},{58,54,4000},{58,56,-16200},{58,58,24400},{58,60,-16200},{58,62,4000},{59,55,4000},{59,57,-16200},{59,59,24400},{59,61,-16200},{59,63,4000},{60,56,4000},{60,58,-16200},{60,60,24400},{60,62,-16200},{60,64,4000},{61,57,4000},{61,59,-16200},{61,61,24400},{61,63,-16200},{61,65,4000},{62,58,4000},{62,60,-16200},{62,62,24400},{62,64,-16200},{62,66,4000},{63,59,4000},{63,61,-16200},{63,63,24400},{63,65,-16200},{63,67,4000},{64,60,4000},{64,62,-16200},{64,64,24400},{64,66,-16200},{64,68,4000},{65,61,4000},{65,63,-16200},{65,65,24400},{65,67,-16200},{65,69,4000},{66,62,4000},{66,64,-16200},{66,66,24400},{66,68,-16200},{66,70,4000},{67,63,4000},{67,65,-16200},{67,67,24400},{67,69,-16200},{67,71,4000},{68,64,4000},{68,66,-16200},{68,68,24400},{68,70,-16200},{68,72,4000},{69,65,4000},{69,67,-16200},{69,69,24400},{69,71,-16200},{69,73,4000},{70,66,4000},{70,68,-16200},{70,70,24400},{70,72,-16200},{70,74,4000},{71,67,4000},{71,69,-16200},{71,71,24400},{71,73,-16200},{71,75,4000},{72,68,4000},{72,70,-16200},{72,72,20400},{72,74,-8200},{73,69,4000},{73,71,-16200},{73,73,20400},{73,75,-8200},{74,70,4000},{74,72,-8200},{74,74,4200},{75,71,4000},{75,73,-8200},{75,75,4200},{76,32,4000},{76,34,-16000},{76,76,24000},{76,78,-16000},{76,80,4000},{77,33,4000},{77,35,-16000},{77,77,24000},{77,79,-16000},{77,81,4000},{78,34,4000},{78,76,-16000},{78,78,24200},{78,80,-16200},{78,82,4000},{79,35,4000},{79,77,-16000},{79,79,24200},{79,81,-16200},{79,83,4000},{80,76,4000},{80,78,-16200},{80,80,24400},{80,82,-16200},{80,84,4000},{81,77,4000},{81,79,-16200},{81,81,24400},{81,83,-16200},{81,85,4000},{82,78,4000},{82,80,-16200},{82,82,24400},{82,84,-16200},{82,86,4000},{83,79,4000},{83,81,-16200},{83,83,24400},{83,85,-16200},{83,87,4000},{84,80,4000},{84,82,-16200},{84,84,24400},{84,86,-16200},{84,88,4000},{85,81,4000},{85,83,-16200},{85,85,24400},{85,87,-16200},{85,89,4000},{86,82,4000},{86,84,-16200},{86,86,24400},{86,88,-16200},{86,90,4000},{87,83,4000},{87,85,-16200},{87,87,24400},{87,89,-16200},{87,91,4000},{88,84,4000},{88,86,-16200},{88,88,24400},{88,90,-16200},{88,92,4000},{89,85,4000},{89,87,-16200},{89,89,24400},{89,91,-16200},{89,93,4000},{90,86,4000},{90,88,-16200},{90,90,24400},{90,92,-16200},{90,94,4000},{91,87,4000},{91,89,-16200},{91,91,24400},{91,93,-16200},{91,95,4000},{92,88,4000},{92,90,-16200},{92,92,24400},{92,94,-16200},{92,96,4000},{93,89,4000},{93,91,-16200},{93,93,24400},{93,95,-16200},{93,97,4000},{94,90,4000},{94,92,-16200},{94,94,24400},{94,96,-16200},{94,98,4000},{95,91,4000},{95,93,-16200},{95,95,24400},{95,97,-16200},{95,99,4000},{96,92,4000},{96,94,-16200},{96,96,24400},{96,98,-16200},{96,100,4000},{97,93,4000},{97,95,-16200},{97,97,24400},{97,99,-16200},{97,101,4000},{98,94,4000},{98,96,-16200},{98,98,24400},{98,100,-16200},{98,102,4000},{99,95,4000},{99,97,-16200},{99,99,24400},{99,101,-16200},{99,103,4000},{100,96,4000},{100,98,-16200},{100,100,24400},{100,102,-16200},{100,104,4000},{101,97,4000},{101,99,-16200},{101,101,24400},{101,103,-16200},{101,105,4000},{102,98,4000},{102,100,-16200},{102,102,24400},{102,104,-16200},{102,106,4000},{103,99,4000},{103,101,-16200},{103,103,24400},{103,105,-16200},{103,107,4000},{104,100,4000},{104,102,-16200},{104,104,24400},{104,106,-16200},{104,108,4000},{105,101,4000},{105,103,-16200},{105,105,24400},{105,107,-16200},{105,109,4000},{106,102,4000},{106,104,-16200},{106,106,24400},{106,108,-16200},{106,110,4000},{107,103,4000},{107,105,-16200},{107,107,24400},{107,109,-16200},{107,111,4000},{108,104,4000},{108,106,-16200},{108,108,24400},{108,110,-16200},{108,112,4000},{109,105,4000},{109,107,-16200},{109,109,24400},{109,111,-16200},{109,113,4000},{110,106,4000},{110,108,-16200},{110,110,24400},{110,112,-16200},{110,114,4000},{111,107,4000},{111,109,-16200},{111,111,24400},{111,113,-16200},{111,115,4000},{112,108,4000},{112,110,-16200},{112,112,20400},{112,114,-8200},{113,109,4000},{113,111,-16200},{113,113,20400},{113,115,-8200},{114,110,4000},{114,112,-8200},{114,114,4200},{115,111,4000},{115,113,-8200},{115,115,4200}};
//}

//std::vector< double > get_b_values()
//{
//  return {-133.493,198.568,-70.0052,108.89,578.067,-256.923,-842.849,-61.5121,343.967,661.812,376.65,-800.358,-368.152,231.29,-276.941,277.849,627.813,-425.445,-495.848,597.819,553.702,-684.566,-935.536,461.081,917.92,-241.278,7.1888,344.709,-1103.24,-906.649,1283.91,1475.66,-714.284,-1358.23,171.817,840.313,-64.5307,205.286,96.5359,-228.372,-26.3433,89.7603,-189.322,-104.807,374.695,215.744,-380.951,-101.999,227.399,-242.303,-123.695,561.607,104.091,-614.284,55.3023,234.217,-168.928,258.119,-175.818,-336.651,764.285,61.3538,-945.915,191.229,617.593,-262.859,-338.846,222.79,285.039,-164.56,-127.053,190.816,-55.128,-218.226,51.7266,95.6422,154.721,-540.663,-176.159,8.87963,-12.6315,546.603,-4.03731,-573.335,109.908,123.43,97.8672,216.961,-361.742,-202.087,403.965,215.064,-435.32,-314.781,433.561,365.138,-121.238,-510.77,-227.542,622.037,245.126,-417.629,-216.248,5.81725,204.085,226.85,-28.1338,-170.709,16.5278,114.771,-170.171,-128.517,127.28,77.4015,-4.01737,-15.3194,-0,-0,-0,-0};
//}

//TEST(EigenSparse, SparseInversion)
//{
//  uint n = 120;
//  Eigen::SparseMatrix<double> A(n, n);
//  // fill A
//  std::vector< Eigen::Triplet< double > > values = get_a_values();
//  A.setFromTriplets(values.begin(), values.end());
//  //

//  Eigen::VectorXd b(n);
//  // fill B
//  auto b_values = get_b_values();
//  for( uint i = 0; i < n; ++i )
//  {
//    b[i] = b_values[i];
//  }
//  //
//  //Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
//  //solver.compute(A);
//  mlr::timerStart();

//  Eigen::VectorXd x;
//  for(uint t = 0; t < 100; ++t)
//  {
//    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(A);
//    x = chol.solve(b);
//  }

//  auto time = mlr::timerPause();

//  std::cout << "time:" << time / 100.0 << " result:" << x << std::endl;
//}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

