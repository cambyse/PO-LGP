#include <Ors/ors.h>
#include <Motion/komo.h>

//===========================================================================

void TEST(Easy){
  ors::KinematicWorld G("test.ors");
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"), 1);
  for(uint i=0;i<5;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(Easy2){
  ors::KinematicWorld G("model.kvg");
  makeConvexHulls(G.shapes);
//  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"), 1);
  for(uint i=0;i<5;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

//  testEasy();
  testEasy2();

  return 0;
}


