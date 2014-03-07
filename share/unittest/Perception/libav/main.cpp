#include <Perception/avutil.h>
#include <gtest/gtest.h>
#include <iostream>

TEST(mt_guess_format, test_std_formats) {
    AVOutputFormat *fmt = mt_guess_format("test.wav", nullptr);
    ASSERT_NE(nullptr, fmt);
    std::clog << fmt->name << std::endl;
    fmt = mt_guess_format("test.avi", nullptr);
    ASSERT_NE(nullptr, fmt);
    std::clog << fmt->name << std::endl;
}


GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
