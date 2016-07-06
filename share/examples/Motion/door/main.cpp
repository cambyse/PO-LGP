#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>
#include <Ors/ors_swift.h>
#include "../../../../usr/PE/src/plotUtil.h"

void TEST(Door){
  ors::KinematicWorld world("model.kvg");
  world.watch(false);
  world.gl().resize(800,800);
  arr X;
  X << FILE("door101/Xdemo.dat");

  arr Pdemo1f,Pdemo2f;
  drawLine(world,X,Pdemo1f,"endeffC1",0,0,X.d0);
  drawLine(world,X,Pdemo2f,"endeffC2",0,0,X.d0);

  write(LIST<arr>(Pdemo1f),STRING("data/P1_0.dat"));
  write(LIST<arr>(Pdemo2f),STRING("data/P2_0.dat"));

  uint idxStart = 70;
  uint idxEnd = 95;
//  drawLine(world,X,Pdemo1f,"endeffC1",2,idxStart,idxEnd);
//  drawLine(world,X,Pdemo2f,"endeffC2",2,idxStart,idxEnd);

  world.gl().camera.X.pos = ors::Vector(2.45698, -1.07052, 1.76987);
  world.gl().camera.X.rot = ors::Quaternion(0.724036, 0.520922,0.270954, 0.361932);

  world.watch(true);
//  world.watch(true);
  cout << world.gl().camera.X.pos << endl;
  cout << world.gl().camera.X.rot << endl;

  displayTrajectory(X,X.d0*2,world,"");
  world.watch(true);

  arr X2;
  uint i=1;
  while (i<22){

//  for (uint i=1;i<22;i++) {
    X2 << FILE(STRING("door101/mbX"<<i<<".dat"));

    arr Pdemo3f,Pdemo4f;
    drawLine(world,X2,Pdemo3f,"endeffC1",2,0,X2.d0);
    drawLine(world,X2,Pdemo4f,"endeffC2",2,0,X2.d0);


    write(LIST<arr>(Pdemo3f),STRING("data/P1_"<<i<<".dat"));
    write(LIST<arr>(Pdemo4f),STRING("data/P2_"<<i<<".dat"));

//    displayTrajectory(X2,X2.d0*2,world,"");
    world.watch(false);
    i=i+1;
  }
  world.watch(true);
}

void TEST(Door2){
  ors::KinematicWorld world("model.kvg");
  world.watch(false);
  world.gl().resize(800,800);
  arr X;
  X << FILE("door101/mfX0.dat");
//  X << FILE("door101/Xdemo.dat");

  arr Pdemo1f,Pdemo2f;
  drawLine(world,X,Pdemo1f,"endeffC1",0,0,X.d0);
  drawLine(world,X,Pdemo2f,"endeffC2",0,0,X.d0);

  write(LIST<arr>(Pdemo1f),STRING("data/P1_0.dat"));
  write(LIST<arr>(Pdemo2f),STRING("data/P2_0.dat"));

  uint idxStart = 70;
  uint idxEnd = 95;
//  drawLine(world,X,Pdemo1f,"endeffC1",2,idxStart,idxEnd);
//  drawLine(world,X,Pdemo2f,"endeffC2",2,idxStart,idxEnd);

  world.gl().camera.X.pos = ors::Vector(2.58159, -0.784158, 1.70426);
  world.gl().camera.X.rot = ors::Quaternion(0.703235, 0.505085, 0.313495, 0.38996);


  world.watch(true);
//  world.watch(true);
  cout << world.gl().camera.X.pos << endl;
  cout << world.gl().camera.X.rot << endl;

  displayTrajectory(X,X.d0*2,world,"");
//  world.watch(true);

  arr X2;
  uint i=1;
  while (i<22){

//  for (uint i=1;i<22;i++) {
    X2 << FILE(STRING("door101/mfX"<<i<<".dat"));

    arr Pdemo3f,Pdemo4f;
    drawLine(world,X2,Pdemo3f,"endeffC1",2,0,X2.d0);
    drawLine(world,X2,Pdemo4f,"endeffC2",2,0,X2.d0);

    write(LIST<arr>(Pdemo3f),STRING("data/P1_"<<i<<".dat"));
    write(LIST<arr>(Pdemo4f),STRING("data/P2_"<<i<<".dat"));

//    displayTrajectory(X2,X2.d0*2,world,"");
    world.watch(false);
    i=i+1;
  }
  world.watch(true);
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  testDoor();
//  testDoor2();
  return 0;
}
