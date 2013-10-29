#include <Core/util.h>
#include <gtest/gtest.h>

//=============================================================================
TEST(Substring, ValidRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getSubString(0, 4), "this");
  EXPECT_EQ(fullString.getSubString(5, 7), "is");
  EXPECT_EQ(fullString.getSubString(10, 14), "long");
  EXPECT_EQ(fullString.getSubString(0, fullString.N), original);
}
TEST(Substring, EndGreaterThanRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getSubString(0, 100000), original);
}

//=============================================================================
TEST(StringGetLastN, ValidRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getLastN(1), "g");
  EXPECT_EQ(fullString.getLastN(2), "ng");
  EXPECT_EQ(fullString.getLastN(3), "ing");
}
TEST(StringGetLastN, RequestTooManyChars) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getLastN(1000), original);
}

//=============================================================================
TEST(StringGetFirstN, ValidRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getFirstN(1), "t");
  EXPECT_EQ(fullString.getFirstN(5), "this ");
}
TEST(StringGetFirstN, RequestTooManyChars) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getFirstN(1000), original);
}

//=============================================================================
GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
