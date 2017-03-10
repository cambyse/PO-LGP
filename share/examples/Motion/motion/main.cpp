#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_transition.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/convert.h>
#include <Optim/lagrangian.h>
#include <Kin/kin_swift.h>

//===========================================================================

void TEST(PR2reach){
  mlr::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));
  makeConvexHulls(G.shapes);
  for(mlr::Shape *s:G.shapes) s->cont=true;
  G.getShapeByName("target")->cont=false;
  cout <<"loaded model: n=" <<G.q.N <<endl;

  MotionProblem MP(G);

  //-- setup the motion problem
  Task *t;

  t = MP.addTask("transitions", new TaskMap_Transition(G), OT_sumOfSqr);
  t->map->order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

//  t = MP.addTask("final_vel", new TaskMap_Transition(G));
  t = MP.addTask("endeff_pos", new TaskMap_Default(posTMT, G, "endeff"), OT_sumOfSqr);
  t->map->order=1; //make this a velocity task!
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

  t = MP.addTask("endeff_pos", new TaskMap_Default(posTMT, G, "endeff", NoVector, NULL, MP.world.getShapeByName("target")->X.pos), OT_sumOfSqr);
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

#define CONSTRAINT
#ifndef CONSTRAINT
  t = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0u}, .1), OT_sumOfSqr);
#else
  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1), OT_ineq);
#endif
  t->setCostSpecs(0, MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization(); //replicate(MP.x0, MP.T+1);

  //-- optimize
  for(uint k=0;k<5;k++){
    mlr::timerStart();
    mlr::KinematicWorld::setJointStateCount=0;
#ifndef CONSTRAINT
    optNewton(x, Convert(MP), OPT(verbose=2, nonStrictSteps=(!k?15:5)));
#else
    optConstrained(x, NoArr, Convert(MP.komo_problem), OPT(verbose=2, stopIters=100, damping=1., maxStep=1., nonStrictSteps=5));
#endif

    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<mlr::KinematicWorld::setJointStateCount <<endl;
    cout <<MP.getReport();
    write(LIST<arr>(x),"z.output");
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, "planned trajectory", 0.01);
  }
}

//===========================================================================

void TEST(Basics){
  mlr::KinematicWorld G("test.ors");
  G.getShapeByName("target")->cont=false;

  MotionProblem MP(G);

  //-- setup the motion problem
  Task *t;

  t = MP.addTask("transitions", new TaskMap_Transition(G), OT_sumOfSqr);
  t->map->order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  //#define CONSTRAINT
  #ifndef CONSTRAINT
  t = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0u}, .1), OT_sumOfSqr);
  #else
  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1), OT_ineq);
  #endif
  t->setCostSpecs(0, MP.T, {0.}, 1.);

  t = MP.addTask("final_vel", new TaskMap_Transition(G), OT_sumOfSqr);
  t->map->order=1; //make this a velocity task!
  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e1);

  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff", mlr::Vector(0, 0, 0), NULL, MP.world.getShapeByName("target")->X.pos), OT_sumOfSqr);
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);


  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config

  //gradient check: will fail in case of collisions
  for(uint k=0;k<0;k++){
    checkJacobian(Convert(MP.komo_problem), x, 1e-4);
    rndUniform(x,-1.,1.);
  }

  //-- optimize
  for(uint k=0;k<5;k++){
    mlr::timerStart();
#ifndef CONSTRAINT
    optNewton(x, Convert(MP));
#else
    optConstrained(x, NoArr, Convert(MP.komo_problem));
#endif
    cout <<"** optimization time=" <<mlr::timerRead() <<endl;
    MP.costReport();
//    checkJacobian(Convert(MF), x, 1e-5);
    write(LIST<arr>(x),"z.output");
    gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, "planned trajectory", 0.01);
  }
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  testPR2reach();
//  testBasics();
  
  return 0;
}


