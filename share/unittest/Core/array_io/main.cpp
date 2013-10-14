#include <Core/array.h>
#include <Core/util.h>
#include <gtest/gtest.h>


TEST(ArrayIO, vector_read_write) {
  arr a = randn(5, 1);
  ofstream of("vec.tmp");
  a.write(of);
  of.close();

  arr b;
  b.read("vec.tmp");

  for (int i = 0; i < 5; ++i) {
      EXPECT_NEAR(a(i, 0), b(i, 0), 0.001);
  }
}


TEST(ArrayIO, matrix_read_write) {
  arr a = randn(5, 5);
  ofstream of("mat.tmp");
  a.write(of);
  of.close();

  arr b;
  b.read("mat.tmp");

  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_NEAR(a(i, j), b(i, j), 0.001);
    }
  }
}


TEST(ArrayIO, tensor_read_write) {
  arr a = randn(5, 25);
  a.reshape(5, 5, 5);
  ofstream of("tensor.tmp");
  of << a;
  of.close();

  arr b;
  ifstream inf("tensor.tmp");
  inf >> b;
  inf.close();

  EXPECT_EQ(a.nd, b.nd);

  // for (int i = 0; i < 5; ++i) {
  //   for (int j = 0; j < 5; ++j) {
  //     for (int k = 0; k < 5; ++k) {
  //       EXPECT_NEAR(a(i, j, k), b(i, j, k), 0.001);
  //     }
  //   }
  // }
}


GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
