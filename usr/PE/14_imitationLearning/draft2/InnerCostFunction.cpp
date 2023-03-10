#include "InnerCostFunction.h"
SimpleICF::SimpleICF(mlr::KinematicWorld world) {
    TaskMap *tm = new TaskMap_Default(posTMT,world,"endeff", mlr::Vector(0., 0., 0.));
    Task *c = new Task(tm);
    c->name = "pos_right_hand";
    taskCosts.append(c);

    TaskMap *tm2 = new TaskMap_Default(vecTMT,world,"endeff", mlr::Vector(0., 1., 0.));
    Task *c2 = new Task(tm2);
    c2->name = "vec_right_hand";
    taskCosts.append(c2);

    TaskMap *tm3 = new TaskMap_qItself();
    Task *c3 = new Task(tm3);
    c3->name = "qItself";
    taskCosts.append(c3);

  }

  void SimpleICF::setParam(const arr &param,const mlr::KinematicWorld &world,const uint T) {
    // pos right hand task
    taskCosts(0)->prec.resize(T+1).setZero();
    taskCosts(0)->prec(T) = param(0);
    taskCosts(0)->target.resize(T+1,3).setZero();
    taskCosts(0)->target[T]() = ARR(world.getBodyByName("goalRef")->X.pos);
    // vec right hand task
    taskCosts(1)->prec.resize(T+1).setZero();
    taskCosts(1)->prec(T) = param(1);
    taskCosts(1)->target.resize(T+1,3).setZero();
    taskCosts(1)->target[T]() = ARR(1.,0.,0.);
    // qItself task
    taskCosts(2)->prec.resize(T+1) = param(2);
    taskCosts(2)->target.resize(T+1,world.getJointStateDimension()).setZero();

    for (uint i=0;i<taskCosts.d0;i++) {
      cout << "prec: "<< taskCosts(i)->prec << endl;
      cout << "target: "<< taskCosts(i)->target << endl;
    }
  }

  //struct GenericICF:InnerCostFunction{
//  GenericICF(bool R_EFF_POS,bool R_EFF_VEC) {
//    if (R_EFF_POS) {
//      taskCosts.append();
//    }
//    //  Task *c;
//    //  c = MP.addTask("position_right_hand", new TaskMap_Default(posTMT,world,"endeff", mlr::Vector(0., 0., 0.)));
//    //  c->setCostSpecs(MP.T, MP.T, refGoal, param(0));
//    //  c = MP.addTask("vec_right_hand", new TaskMap_Default(vecTMT,world,"endeff", mlr::Vector(0., 1., 0.)));
//    //  c->setCostSpecs(MP.T, MP.T, ARR(0.,1.,0.), param(1));
//    //  c = MP.addTask("final_vel", new TaskMap_qItself());
//    //  MP.setInterpolatingCosts(c,KOMO::finalOnly,{0.},param(2));
//    //  c->map.order=1;
//  }

////  void setParam(const arr &param) {
////    for(uint i=0; i<taskCosts.N; i++) {
////      taskCosts(i)->prec = param;
////    }
////  }

//  void setTarget(const mlr::KinematicWorld &world) {

//  }
//};
