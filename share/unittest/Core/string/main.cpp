#include <Core/util.h>
#include <gtest/gtest.h>

//=============================================================================
GTEST_TEST(Substring, ValidRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getSubString(0, 4), "this");
  EXPECT_EQ(fullString.getSubString(5, 7), "is");
  EXPECT_EQ(fullString.getSubString(10, 14), "long");
  EXPECT_EQ(fullString.getSubString(0, fullString.N), original);
}
GTEST_TEST(Substring, EndGreaterThanRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getSubString(0, 100000), original);
}

//=============================================================================
GTEST_TEST(StringGetLastN, ValidRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getLastN(1), "g");
  EXPECT_EQ(fullString.getLastN(2), "ng");
  EXPECT_EQ(fullString.getLastN(3), "ing");
}
GTEST_TEST(StringGetLastN, RequestTooManyChars) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getLastN(1000), original);
}

//=============================================================================
GTEST_TEST(StringGetFirstN, ValidRange) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getFirstN(1), "t");
  EXPECT_EQ(fullString.getFirstN(5), "this ");
}
GTEST_TEST(StringGetFirstN, RequestTooManyChars) {
  const char* original  = "this is a long string";
  MT::String fullString(original);
  EXPECT_EQ(fullString.getFirstN(1000), original);
}

//=============================================================================
GTEST_TEST(StringStartsWith, PosExamples) {
  MT::String fullString("this is a long string");

  EXPECT_TRUE(fullString.startsWith("t"));
  EXPECT_TRUE(fullString.startsWith(String("t")));

  EXPECT_TRUE(fullString.startsWith("this"));
  EXPECT_TRUE(fullString.startsWith(String("this")));

  EXPECT_TRUE(fullString.startsWith("this is"));
  EXPECT_TRUE(fullString.startsWith(String("this is")));
}

GTEST_TEST(StringStartsWith, NegExamples) {
  MT::String fullString("this is a long string");

  EXPECT_FALSE(fullString.startsWith(("b")));
  EXPECT_FALSE(fullString.startsWith(String("b")));

  EXPECT_FALSE(fullString.startsWith("is"));
  EXPECT_FALSE(fullString.startsWith(String("is")));

  EXPECT_FALSE(fullString.startsWith("ntiearnsto"));
  EXPECT_FALSE(fullString.startsWith(String("ntiearnsto")));
}

//=============================================================================
GTEST_TEST(StringEndsWith, PosExamples) {
  MT::String fullString("this is a long string");

  EXPECT_TRUE(fullString.endsWith("g"));
  EXPECT_TRUE(fullString.endsWith(String("g")));

  EXPECT_TRUE(fullString.endsWith("string"));
  EXPECT_TRUE(fullString.endsWith(String("string")));
}

GTEST_TEST(StringEndsWith, NegExamples) {
  MT::String fullString("this is a long string");

  EXPECT_FALSE(fullString.endsWith("somethig"));
  EXPECT_FALSE(fullString.endsWith(String("something")));
}

//=============================================================================
GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
