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

  a.reshape(25);
  b.reshape(25);
  for (int i = 0; i < 25; ++i) {
    EXPECT_NEAR(a(i), b(i), 0.001);
  }
}

TEST(ArrayIO, tensor_read_write) {
  arr a = randn(5, 25);
  a.reshape(5, 5, 5);
  ofstream of("tensor.tmp");
  a.write(of);
  of.close();

  arr b;
  ifstream inf("tensor.tmp");
  inf >> b;
  inf.close();

  EXPECT_EQ(a.nd, b.nd);

  a.reshape(125);
  b.reshape(125);
  for (int i = 0; i < 125; ++i) {
    EXPECT_NEAR(a(i), b(i), 0.001);
  }
}

TEST(ArrayIO, tensor_high_dim_read_write) {
  arr a = randn(5, 125);
  a.reshape(TUP(5, 5, 5, 5));
  ofstream of("tensor_hd.tmp");
  a.write(of);
  of.close();

  arr b;
  ifstream inf("tensor_hd.tmp");
  inf >> b;
  inf.close();

  EXPECT_EQ(a.nd, b.nd);

  a.reshape(625);
  b.reshape(625);
  for (int i = 0; i < 625; ++i) {
    EXPECT_NEAR(a(i), b(i), 0.001);
  }
}


GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
