#include <Ors/ors.h>
#include <Gui/mesh.h>
#include <gtest/gtest.h>

//=============================================================================
GTEST_TEST(Ors, getSubMeshPos_numberOfParts) {
  const char* filename = "example.obj";
  auto positions = getSubMeshPositions(filename);

  FILE* file;
  file = fopen(filename, "r");
  if (!file) HALT("readObjFile() failed: can't open data file" << filename);

  int num_parts = 0;
  for (auto p : positions) {
    // cout << "reading from " << std::get<0>(p) << " to " << std::get<1>(p) << endl;
    fseek(file, std::get<0>(p), SEEK_SET);

    char buf[128];
    while ((fscanf(file, "%s", buf) != EOF) && (ftell(file) < std::get<1>(p)))
    {
      // cout << buf << " ";
    }
    // cout << "\n" << endl;
    num_parts++;
  }
  EXPECT_EQ(num_parts, 3) << "We expect three parts";
}

//=============================================================================
GTEST_TEST(Ors, ParseObjFirstPart) {
  ors::Mesh mesh;
  mesh.parsing_pos_start = 0;
  mesh.parsing_pos_end = 50;
  mesh.readObjFile(FILE("example.obj"));

  // check V dimensions
  EXPECT_EQ(mesh.V.d0, 3);
  EXPECT_EQ(mesh.V.d1, 3);

  // check V content
  arr V_correct = {1.1, 1.2, 1.3, 2.1, 2.2, 2.3, 3.1, 3.2, 3.3};
  V_correct.reshape(3, 3);
  EXPECT_EQ(mesh.V, V_correct);

  // check T
  uintA T_correct = {0, 1, 2};
  T_correct.reshape(1, 3);
  EXPECT_EQ(mesh.T, T_correct);
}

GTEST_TEST(Ors, ParseObjSecondPart) {
  ors::Mesh mesh;
  mesh.parsing_pos_start = 50;
  mesh.parsing_pos_end = 100;
  mesh.readObjFile(FILE("example.obj"));

  // check V dimensions
  EXPECT_EQ(mesh.V.d0, 3);
  EXPECT_EQ(mesh.V.d1, 3);

  // check V content
  arr V_correct = {4.1, 4.2, 4.3, 5.1, 5.2, 5.3, 6.1, 6.2, 6.3};
  V_correct.reshape(3, 3);
  EXPECT_EQ(mesh.V, V_correct);

  // check T
  uintA T_correct = {0, 1, 2};
  T_correct.reshape(1, 3);
  EXPECT_EQ(mesh.T, T_correct);

}
//=============================================================================
GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
