#include <Motion/komo.h>
#include <string>
#include <map>
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>

using namespace std;

//===========================================================================

void TEST(KomoSequence){
  
  mlr::KinematicWorld W("model.g");
  mlr::KinematicWorld Wfin("model-final.g");
  KOMO komo;
  komo.setModel(W);

  komo.setTiming(4., 20, 5., 2, false);
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchVelocities();
  komo.setSquaredQAccelerations();

  komo.displayCamera().setPosition(-5.,-1.,2.);
  komo.displayCamera().focus(0,0,1.);
  komo.displayCamera().upright();

  // better use low-level GJK instead of SWIFT
//  komo.MP->world.swift().activate(komo.MP->world.getShapeByName("yellow"));
//  komo.MP->world.swift().activate(komo.MP->world.getShapeByName("red"));

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanR", "blue");
//  komo.setPlace(1.8, "humanR", "blue", "tableC");
  komo.setPlace(2.5, "humanR", "blue", "red");

  komo.setGrasp(1.5, "humanL", "yellow");
//  komo.setPlace(2.1, "humanL", "yellow", "tableC");
  komo.setPlace(3., "humanL", "yellow", "blue");

  //low-level implementation of distance, using GJK and inequality (>5cm)
  komo.setTask(1.5,2.5, new TaskMap_GJK(komo.world, "yellow", "blue", true, true), ineqTT, ARR(-.05), 1e2, 0);


//  komo.setAlignedStacking(2.2, "yellow");
//  komo.setAlignedStacking(2.2, "blue");




  komo.reset();
  komo.run();
//  komo.checkGradients();

  cout <<komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha=1.;

  testKomoSequence();

  return 0;
}

