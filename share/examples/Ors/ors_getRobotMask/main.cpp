#include <Ors/ors.h>
#include <Gui/opengl.h>


void TEST(GetRobotMask){
  ors::KinematicWorld robot("pr2_model/pr2_model.ors");
  robot.gl().Reshape(580,480);
  robot.gl().camera.setKinect();
  robot.gl().camera.X = robot.getShapeByName("endeffKinect")->X * robot.gl().camera.X;
//  robot.gl().watch(); //if commented, glut/gtk is never initiated
  byteA indexRgb, depth;
  robot.glGetMasks(indexRgb, depth);
  write_ppm(indexRgb, "z.rgb.ppm");
  write_ppm(depth, "z.depth.ppm");
}

// ============================================================================

int main(int argc, char** argv) {
  testGetRobotMask();

  return 0;
}


