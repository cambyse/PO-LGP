#include "swig.h"

#include "lockbox/lockbox.h"

struct sLockboxSwig{
  MyBaxter B;
  Lockbox L;

  sLockboxSwig();

};



LockboxSwig::LockboxSwig()
  :s (new sLockboxSwig){
}

LockboxSwig::~LockboxSwig(){
  delete s;
}

bool LockboxSwig::testJoint(const int jointNumber){

  return s->L.testJoint(jointNumber);
}

double LockboxSwig::getJointPosition(const int jointNumber){
  return s->L.getJointPosition(jointNumber);
}


//void LockboxSwig::testArray(std::vector<double> _x){
//  arr x = conv_stdvec2arr(_x);
//  cout <<"i received an array: " <<x <<endl;
//}




sLockboxSwig::sLockboxSwig():L(&B){
  L.initializeJoints(); // Currently keeping this separate, since it doesn't wait for all modules to be really started...

  L.moveHome(true);

  if (mlr::getParameter<bool>("useRos", false))
  {
    B.disablePosControl();
    mlr::wait(5.);
    L.update();
    B.enablePosControl();

//    ors::Transformation tf = B.getModelWorld().getShapeByName("alvar_10")->X;


//    // Position task
//    mlr::String str;
//    ors::Vector target = tf*ors::Vector(-0.05, 0, 0.25);
//    str << "map=pos ref1=endeffL ref2=base_footprint vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1.2, .2, 1.]";

//    auto approach = B.task(GRAPH(str));

//    ors::Vector vecx = tf.rot * ors::Vector(0,0,-1); vecx.normalize();
//    ors::Vector vecy = tf.rot * ors::Vector(1,0,0); vecy.normalize();
//    ors::Vector vecz = tf.rot * ors::Vector(0,-1, 0); vecz.normalize();
//    auto alignX = B.task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[" << vecx.x << ' ' << vecx.y << ' ' << vecx.z << "] target=[1] prec=[1000]  PD=[1., 1.2, .2, 1.]")));
//    auto alignY = B.task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[" << vecy.x << ' ' << vecy.y << ' ' << vecy.z << "] target=[1] prec=[1000] PD=[1., 1.2, .2, 1.]")));
//    auto alignZ = B.task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[" << vecz.x << ' ' << vecz.y << ' ' << vecz.z << "] target=[1] prec=[1000] PD=[1., 1.2, .2, 1.]")));

//    B.waitConv({approach, alignX, alignY, alignZ});
//    B.testRealConv({approach, alignX, alignY, alignZ}, 10);

//    // Now we are 15 cm away and aligned with the handle.
//    B.stop({approach, alignX, alignY, alignZ});

//    B.disablePosControl();
//    mlr::wait(5.);
//    L.update();
//    B.enablePosControl();

//    L.moveHome(true);
  }
  else
  {
    L.update();
  }

  L.readyToTest = true;
}
