#include <Ors/ors.h>
#include <gtest/gtest.h>

#define TEST_DIFF_ZERO(expr) { \
  double e=(expr).diffZero(); \
  EXPECT_LE(e, 1e-6) << " Error="<<e <<" Expression=" <<(expr); }

TEST(OrsTest, testQuaternions) {
  for(uint k=0;k<10;k++){
    ors::Quaternion A,B,C;
    A.setRandom();
    B.setRandom();
    C.setRandom();
    TEST_DIFF_ZERO(Quaternion_Id);
    TEST_DIFF_ZERO(A/A);
    TEST_DIFF_ZERO(A*B/B/A);
  }
}

TEST(OrsTest, testTransformations) {
  for(uint k=0;k<10;k++){
    ors::Transformation A,B,C;
    A.setRandom();
    B.setRandom();
    C.setDifference(A,B);
    TEST_DIFF_ZERO(A*C/B);
  }
}

TEST(OrsTest, testGraph) {
  {
    ors::KinematicWorld G;
    G.init("world.ors");
  }
  {
    ors::KinematicWorld *G2 = new ors::KinematicWorld();
    G2->init("world-complex.ors");
    delete G2;
  }
}

GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
