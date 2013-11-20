#include <Algo/rrt.h>
#include <gtest/gtest.h>

#define TEST_ALMOST_EQUAL(arr1, arr2, prec) { \
  arr arr3 = arr2-arr1; \
  for (double d: arr3) \
    EXPECT_LE(d, prec); \
  }

void test_rrt(long num) {
  double rnd_thresh = .5;
  RRT rrt({0, 0, 0}, .1);

  arr goal = {1., 1.001, 1};

  MT::Rnd r;

  for(long i=0; i<num; ++i) {
    arr next;
    if (r.uni() < rnd_thresh) {
      next = rand(3, 1);
      next.reshape(3);
    }
    else {
      next = goal;
    }


    arr next_before = next;
    double dist = rrt.getProposalTowards(next);

    if(dist < 10e-5) {
      continue;
    }

    arr nearest = rrt.getNode(rrt.getNearest());
    arr next_after = next - nearest;
    next_before = next_before - nearest;

    arr next_normalized_before = next_before/length(next_before);
    arr next_normalized_after = next_after/length(next_after);

    TEST_ALMOST_EQUAL(next_normalized_before, next_normalized_after, .01);
    EXPECT_LE(length(next_after) - rrt.getStepsize(), 0.01);

    uint num_before = rrt.getNumberNodes();
    rrt.add(next);
    uint num_after = rrt.getNumberNodes();

    EXPECT_EQ(num_before, num_after-1);
  }
}

TEST(AlgosTest, testRRTSmall) {
  test_rrt(3);  
}

TEST(AlgosTest, testRRTBig) {
  test_rrt(50000);
}


GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
