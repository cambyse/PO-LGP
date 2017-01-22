#include <Motion/komo.h>
#include <string>
#include <map>
#include <Gui/opengl.h>
#include <Kin/kin_swift.h>
#include <Motion/taskMaps.h>

#include "physx.h"

using namespace std;

#define collisionsOff(x) komo.MP->world.swift().deactivate(komo.MP->world.getShapeByName(x))
#define collisionsOn(x) komo.MP->world.swift().activate(komo.MP->world.getShapeByName(x))


//===========================================================================




void init(KOMO& komo, uint trial, mlr::KinematicWorld& W, mlr::KinematicWorld& Wfin, double phases=4.){
  W.init(STRING("model_"<<trial <<".g"));
  Wfin.init(STRING("model_"<<trial+1 <<".g"));  

  komo.setModel(W);

  komo.setTiming(phases, 20, 5., 2, true);
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  komo.displayCamera().setPosition(-5.,-1.,2.);
  komo.displayCamera().focus(0,0,1.);
  komo.displayCamera().upright();

  // explicitly active certain collision computations (by SWIFT)
  komo.MP->world.swift().deactivate(komo.MP->world.getShapeByName("table"));
 }



//===========================================================================

void optimize(KOMO& komo){
  komo.reset();
  komo.run();
//  komo.checkGradients();

  cout <<komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

void trial1(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin; //intital config W, final config Wfin
 
  init(komo, 1, W, Wfin);
  
  collisionsOn("red");
  collisionsOn("blue");
  collisionsOn("yellow");

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), OT_sumOfSqr, Wfin.getJointState(), 1e3, 0);


  komo.setGrasp(1., "humanR", "red", 0, .8);
  komo.setGrasp(1.5, "humanL", "yellow", 0, .8);
  komo.setPlace(2.5, "humanR", "red", "table");
  komo.setPlace(3., "humanL", "yellow", "table");

  //komo.setPlaceFixed(2.5, "humanR", "red", "table", Wfin.getShapeByName("red")->X/Wfin.getShapeByName("table")->X);
  //komo.setPlaceFixed(3., "humanL", "yellow", "table", Wfin.getShapeByName("yellow")->X/Wfin.getShapeByName("table")->X);

  //low-level implementation of distance, using GJK and inequality (>5cm)
//  komo.setTask(1.5,3., new TaskMap_GJK(komo.world, "yellow", "red", true, true), OT_ineq, ARR(-.02), 1e2, 0);
  komo.setTask(1., 3., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}

//===========================================================================


void trial2(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;
  init(komo, 2, W, Wfin);
  collisionsOn("red");
  collisionsOn("blue");
  collisionsOn("yellow");
  
  komo.setTask(3.9,4., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);
 
  komo.setGrasp(1., "humanR", "red", 0, 0.8); //weight?
  komo.setPlaceFixed(2.0, "humanR", "red", "blue", Wfin.getShapeByName("red")->X/Wfin.getShapeByName("blue")->X );
  
  komo.setTask(1., 3., new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);

  optimize(komo);
}

void trial3(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;

  init(komo, 3, W, Wfin);

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), OT_sumOfSqr, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanR", "red", 0, .8);
  komo.setGrasp(1.5, "humanL", "blue", 0, .8);

  komo.setPlaceFixed(2.5, "humanR", "red", "table", Wfin.getShapeByName("red")->X/Wfin.getShapeByName("table")->X);
  komo.setPlaceFixed(3., "humanL", "blue", "red", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("red")->X);

  //low-level implementation of distance, using GJK and inequality (>5cm)
//  komo.setTask(1.5,3., new TaskMap_GJK(komo.world, "yellow", "red", true, true), OT_ineq, ARR(-.02), 1e2, 0);
  komo.setTask(1.5, 2.5, new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}

//===========================================================================

void trial3b(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;

  init(komo, 3, W, Wfin, 10.);

  //-- fix last configuration
  komo.setTask(9.9, 10., new TaskMap_qItself(), OT_sumOfSqr, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanR", "red", 0, .8);
  komo.setPlace(2., "humanR", "red", "table");
  komo.setGrasp(3., "humanR", "blue", 0, .0);
  komo.setPlace(4., "humanR", "blue", "table");
  komo.setGrasp(5., "humanR", "red", 0, .8);
  komo.setPlaceFixed(6., "humanR", "red", "table", Wfin.getShapeByName("red")->X/Wfin.getShapeByName("table")->X);
  komo.setGrasp(7., "humanR", "blue", 0, .8);
  komo.setPlaceFixed(8., "humanR", "blue", "red", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("red")->X);

  komo.setTask(3., 5., new TaskMap_Proxy(pairsPTMT, {W.getShapeByName("red")->index, W.getShapeByName("blue")->index}, .03), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}

//===========================================================================

void trial3c(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;

  init(komo, 3, W, Wfin, 7.);

  //-- fix last configuration
  komo.setTask(6.9, 7., new TaskMap_qItself(), OT_sumOfSqr, Wfin.getJointState(), 1e3, 0);

#if 0
  komo.setGrasp(1., "humanR", "blue", 0, .0);
  komo.setPlace(2., "humanR", "blue", "table");
#else
  komo.setGraspSlide(1., 2., "humanR", "blue", "table", 0, .0);
#endif
  komo.setGrasp(3., "humanR", "red", 0, .8);
  komo.setPlaceFixed(4., "humanR", "red", "table", Wfin.getShapeByName("red")->X/Wfin.getShapeByName("table")->X);
  komo.setGrasp(5., "humanR", "blue", 0, .8);
  komo.setPlaceFixed(6., "humanR", "blue", "red", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("red")->X);

//  komo.setTask(3., 5., new TaskMap_Proxy(pairsPTMT, {W.getShapeByName("red")->index, W.getShapeByName("blue")->index}, .03), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}

//===========================================================================


void trial4(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;
  
  init(komo, 4, W, Wfin, 6.);
  
  collisionsOff("blue");
  collisionsOff("yellow");
  collisionsOff("red");
  
  komo.setTask(5.9,6., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);
  
  komo.setGrasp(1., "humanL", "blue", 0, .8);
  //komo.setplace(2.5, "humanr", "blue", "table");
  //komo.setgrasp(3.5, "humanl", "blue", 0, 0.8);

  komo.setTask(2., 3., new TaskMap_Default(posDiffTMT , W, "blue", NoVector, "table", NoVector), sumOfSqrTT, NoArr, 1e2, 1);
  komo.setTask(2., 3., new TaskMap_Default(quatDiffTMT, W, "blue", NoVector, "table", NoVector), sumOfSqrTT, NoArr, 1e2, 1);

  komo.setPlaceFixed(3., "humanL", "blue", "table", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("table")->X);

  komo.setTask(1., 5., new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);

  optimize(komo);
} 


void trial5(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin; //intital config W, final config Wfin
 
  init(komo, 5, W, Wfin);
  
  collisionsOn("red");
  collisionsOn("blue");
  collisionsOn("yellow");

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);


  komo.setGrasp(1., "humanL", "blue", 0, .8);
  komo.setPlaceFixed(2., "humanL", "blue", "table", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("table")->X);
 
  komo.setTask(1., 2.5, new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);

  optimize(komo);
}



void trial10(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;
  //collisionsOn("red");
  //collisionsOn("blue");
  //collisionsOn("yellow");


  init(komo, 10, W, Wfin);

  //-- fix last configuration
  komo.setTask(3.9, 4., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanL", "blue", 0, .8);
  komo.setGrasp(1.5, "humanR", "yellow", 0, .8);

  komo.setPlaceFixed(2.5, "humanL", "blue", "red", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("red")->X);
  komo.setPlaceFixed(3., "humanR", "yellow", "blue", Wfin.getShapeByName("yellow")->X/Wfin.getShapeByName("blue")->X);

  //low-level implementation of distance, using GJK and inequality (>5cm)
//  komo.setTask(1.5,3., new TaskMap_GJK(komo.world, "yellow", "red", true, true), ineqTT, ARR(-.02), 1e2, 0);
//  komo.setTask(1., 3., new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);

  optimize(komo);
}




void trial11(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;
  //collisionsOn("red");
  //collisionsOn("blue");
  //collisionsOn("yellow");


  init(komo, 11, W, Wfin);

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanL", "yellow", 0, .8);
  komo.setGrasp(1.5, "humanR", "blue", 0, .8);

  komo.setPlaceFixed(2.5, "humanR", "blue", "red", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("red")->X);
  komo.setPlaceFixed(3., "humanL", "yellow", "blue", Wfin.getShapeByName("yellow")->X/Wfin.getShapeByName("blue")->X);

  //low-level implementation of distance, using GJK and inequality (>5cm)
//  komo.setTask(1.5,3., new TaskMap_GJK(komo.world, "yellow", "red", true, true), ineqTT, ARR(-.02), 1e2, 0);
  komo.setTask(1., 3., new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);

  optimize(komo);
}



void trial27(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;

  init(komo, 27, W, Wfin);

  collisionsOn("yellow");
  collisionsOn("blue");

  //komo.MP->world.swift().deactivate(komo.MP->world.getShapeByName("red"), komo.MP->world.getShapeByName("blue"));
  //komo.MP->world.swift().deactivate(komo.MP->world.getShapeByName("red"), komo.MP->world.getShapeByName("yellow"));
 


 // better use low-level GJK instead of SWIFT
//  komo.MP->world.swift().activate(komo.MP->world.getShapeByName("yellow"));
//  komo.MP->world.swift().activate(komo.MP->world.getShapeByName("red"));

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), OT_sumOfSqr, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanR", "blue", 0, .1);
//  komo.setPlace(1.8, "humanR", "blue", "tableC");
  komo.setPlaceFixed(2.5, "humanR", "blue", "red", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("red")->X);

  komo.setGrasp(1.5, "humanL", "yellow", 0, .1);
//  komo.setPlace(2.1, "humanL", "yellow", "tableC");
  komo.setPlaceFixed(3., "humanL", "yellow", "blue", Wfin.getShapeByName("yellow")->X/Wfin.getShapeByName("blue")->X);

  //low-level implementation of distance, using GJK and inequality (>5cm)
//  komo.setTask(1.5, 2.5, new TaskMap_GJK(komo.world, "yellow", "blue", true, true), OT_ineq, ARR(-.05), 1e2, 0);
  komo.setTask(1., 3., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);

  optimize(komo);
}




void trial33(){
  KOMO komo;
  mlr::KinematicWorld W, Wfin;

  init(komo, 33, W, Wfin);

  collisionsOn("yellow");
  collisionsOn("blue");

  //komo.MP->world.swift().deactivate(komo.MP->world.getShapeByName("red"), komo.MP->world.getShapeByName("blue"));
  //komo.MP->world.swift().deactivate(komo.MP->world.getShapeByName("red"), komo.MP->world.getShapeByName("yellow"));
 


 // better use low-level GJK instead of SWIFT
//  komo.MP->world.swift().activate(komo.MP->world.getShapeByName("yellow"));
//  komo.MP->world.swift().activate(komo.MP->world.getShapeByName("red"));

  //-- fix last configuration
  komo.setTask(3.9,4., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);

  //komo.setGrasp(1., "humanR", "red", 0, .1);
//  komo.setPlace(1.8, "humanR", "blue", "tableC");
// 
  komo.setGrasp(1, "humanL",  "yellow", 0, .1 );
  //komo.setPlace()
  komo.setPlaceFixed(2.5, "humanL", "yellow", "blue", Wfin.getShapeByName("yellow")->X/Wfin.getShapeByName("blue")->X);

//  komo.setGrasp(1.5, "humanL", "yellow", 0, .1);
//  komo.setPlace(2.1, "humanL", "yellow", "tableC");
  //komo.setPlaceFixed(3., "humanR", "red", "yellow", Wfin.getShapeByName("red")->X/Wfin.getShapeByName("yellow")->X);

  //low-level implementation of distance, using GJK and inequality (>5cm)
//  komo.setTask(1.5, 2.5, new TaskMap_GJK(komo.world, "yellow", "blue", true, true), ineqTT, ARR(-.05), 1e2, 0);
  komo.setTask(1., 2.5, new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);

  optimize(komo);
}



//===========================================================================

void testPhysX(){
#if 1
  //============= CUT copied from tria4, needs refactoring to clean the code!!
  KOMO komo;
  mlr::KinematicWorld W, Wfin;

  init(komo, 4, W, Wfin, 6.);

  collisionsOff("blue");
  collisionsOff("yellow");
  collisionsOff("red");

  komo.setTask(5.9,6., new TaskMap_qItself(), sumOfSqrTT, Wfin.getJointState(), 1e3, 0);

  komo.setGrasp(1., "humanL", "blue", 0, .8);
  //komo.setplace(2.5, "humanr", "blue", "table");
  //komo.setgrasp(3.5, "humanl", "blue", 0, 0.8);

  komo.setTask(2., 3., new TaskMap_Default(posDiffTMT , W, "blue", NoVector, "table", NoVector), sumOfSqrTT, NoArr, 1e2, 1);
  komo.setTask(2., 3., new TaskMap_Default(quatDiffTMT, W, "blue", NoVector, "table", NoVector), sumOfSqrTT, NoArr, 1e2, 1);

  komo.setPlaceFixed(3., "humanL", "blue", "table", Wfin.getShapeByName("blue")->X/Wfin.getShapeByName("table")->X);

  komo.setTask(1., 5., new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);
  //============= CUT end


  komo.reset();
  komo.run();

  cout <<komo.getReport(false);

  mlr::KinematicWorld& K = *komo.MP->configurations.last();
#else
  mlr::KinematicWorld K("model_physx.g");
#endif

  runPhysX(K, 1.);
}


//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha=1.;

  testPhysX();
  return 0;

// trial1(); 
//  trial2();
  trial4();
//  trial5();
//  trial3();
//  trial3b();
//  trial3c();
//  trial10();
//  trial11();
//  trial27();
//  trial33();

  return 0;
}

