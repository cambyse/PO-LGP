#include "komo.h"
#include "motion.h"
#include <Algo/spline.h>
#include <iomanip>
#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>

//===========================================================================

void setTasks(MotionProblem& MP,
              ors::Shape &endeff,
              ors::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

//===========================================================================

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

//===========================================================================

double height(ors::Shape* s){
  CHECK(s,"");
  return 2.*s->size[2] + s->size[3];
}

KOMO::KOMO() : MP(NULL){
}

KOMO::KOMO(const Graph& specs) : MP(NULL){
  init(specs);
//  reset();
//  CHECK(x.N,"");
}

void KOMO::init(const Graph& _specs){
  specs = _specs;

  Graph &glob = specs.get<Graph>("KOMO");
  stepsPerPhase=glob.get<double>("T");
  double duration=glob.get<double>("duration");
  maxPhase=glob.get<double>("phases", 1);
  uint k_order=glob.get<double>("k_order", 2);

  if(glob["model"]){
    mlr::FileToken model = glob.get<mlr::FileToken>("model");
    world.read(model);
  }else{
    world.init(specs);
  }

  if(glob["meldFixedJoints"]){
    world.meldFixedJoints();
    world.removeUselessBodies();
  }

  if(glob["makeConvexHulls"])
    makeConvexHulls(world.shapes);

  if(glob["makeSSBoxes"]){
    NIY;
    //for(ors::Shape *s: world.shapes) s->mesh.makeSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(glob["activateAllContacts"])
    for(ors::Shape *s:world.shapes) s->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;

  if(MP) delete MP;
  MP = new MotionProblem(world);
  if(stepsPerPhase>=0) MP->setTiming(stepsPerPhase*maxPhase, duration*maxPhase);
  MP->k_order=k_order;

  MP->parseTasks(specs, stepsPerPhase);
}

void KOMO::setFact(const char* fact){
  specs.readNode(STRING(fact));
  MP->parseTask(specs.last());
}

void KOMO::setModel(const ors::KinematicWorld& W,
                    bool meldFixedJoints, bool makeConvexHulls, bool makeSSBoxes, bool activateAllContacts){

  world.copy(W);

  if(meldFixedJoints){
    world.meldFixedJoints();
    world.removeUselessBodies();
  }

  if(makeConvexHulls){
    ::makeConvexHulls(world.shapes);
  }
  computeMeshNormals(world.shapes);

  if(makeSSBoxes){
    NIY;
    //for(ors::Shape *s: world.shapes) s->mesh.makeSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(activateAllContacts)
    for(ors::Shape *s:world.shapes) s->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint k_order){
  if(MP) delete MP;
  MP = new MotionProblem(world);
  maxPhase = _phases;
  stepsPerPhase = _stepsPerPhase;
  if(stepsPerPhase>=0) MP->setTiming(stepsPerPhase*maxPhase, durationPerPhase*maxPhase);
  MP->k_order=k_order;
}


//===========================================================================
//
// task specs
//

Task *KOMO::setTask(double startTime, double endTime, TaskMap *map, TermType type, const arr& target, double prec, uint order){
  map->order = order;
  Task *task = new Task(map, type);
  task->name <<map->shortTag(MP->world);
  if(endTime>double(maxPhase)+1e-10) LOG(-1) <<"beyond the time!";
  uint tFrom = (startTime<0.?0:STEP(startTime)+order);
  uint tTo = (endTime<0.?MP->T:STEP(endTime));
  if(tFrom>tTo && tFrom-tTo<=order) tFrom=tTo;
  task->setCostSpecs(tFrom, tTo, target, prec);
  MP->tasks.append(task);
  return task;
}

Task *KOMO::setTask(double startTime, double endTime, const char* mapSpecs, TermType type, const arr& target, double prec, uint order){
  TaskMap *map = TaskMap::newTaskMap(Graph(mapSpecs), world);
  return setTask(startTime, endTime, map, type, target, prec, order);
}

void KOMO::setKinematicSwitch(double time, bool before, const char* type, const char* ref1, const char* ref2, const ors::Transformation& jFrom, const ors::Transformation& jTo){
  ors::KinematicSwitch *sw = ors::KinematicSwitch::newSwitch(type, ref1, ref2, world, STEP(time)+(before?-1:0), 0, jFrom, jTo );
  MP->switches.append(sw);
}

void KOMO::setHoming(double startTime, double endTime, double prec){
  setTask(startTime, endTime, new TaskMap_qItself(NoArr, true), sumOfSqrTT, NoArr, prec); //world.q, prec);
}

void KOMO::setSquaredQAccelerations(double startTime, double endTime, double prec){
  CHECK(MP->k_order>=2,"");
  setTask(startTime, endTime, new TransitionTaskMap(MP->world), sumOfSqrTT, NoArr, prec, 2);
}

void KOMO::setSquaredQVelocities(double startTime, double endTime, double prec){
  CHECK(MP->k_order>=1,"");
  auto *map = new TransitionTaskMap(MP->world);
  setTask(startTime, endTime, map, sumOfSqrTT, NoArr, prec, 1);
  map->velCoeff = 1.;
}

void KOMO::setSquaredFixJointVelocities(double startTime, double endTime, double prec){
  CHECK(MP->k_order>=1,"");
  auto *map = new TransitionTaskMap(MP->world, true);
  setTask(startTime, endTime, map, sumOfSqrTT, NoArr, prec, 1);
  map->velCoeff = 1.;
}

void KOMO::setHoldStill(double startTime, double endTime, const char* joint, double prec){
  setTask(startTime, endTime, new TaskMap_qItself(world, joint), sumOfSqrTT, NoArr, prec, 1);
}

void KOMO::setPosition(double startTime, double endTime, const char* shape, const char* shapeRel, TermType type, const arr& target, double prec){
#if 0
  mlr::String map;
  map <<"map=pos ref1="<<shape;
  if(shapeRel) map <<" ref2=" <<shapeRel;
  setTask(startTime, endTime, map, type, target, prec);
#else
  setTask(startTime, endTime, new DefaultTaskMap(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);
#endif
}

void KOMO::setLastTaskToBeVelocity(){
  MP->tasks.last()->map.order = 1; //set to be velocity!
}

void KOMO::setGrasp(double time, const char* endeffRef, const char* object, bool effKinMode){
//  mlr::String& endeffRef = world.getShapeByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;

  //-- position the hand & graspRef
#if 0
  ors::Body *endeff = world.getShapeByName(endeffRef)->body;
  mlr::String& graspRef = endeff->outLinks.last()->to->shapes.scalar()->name;
  setTask(time-.1, time, new DefaultTaskMap(vecTMT, world, endeffRef, Vector_z), sumOfSqrTT, {0.,0.,1.}, 1e3);
  setTask(time-.1, time, new DefaultTaskMap(posDiffTMT, world, graspRef, NoVector, object, NoVector), sumOfSqrTT, NoArr, 1e3);
  setTask(time-.1, time, new DefaultTaskMap(quatDiffTMT, world, graspRef, NoVector, object, NoVector), sumOfSqrTT, NoArr, 1e3);
#else
  setTask(time-.1, time, new DefaultTaskMap(vecTMT, world, endeffRef, Vector_z), sumOfSqrTT, {0.,0.,1.}, 1e3);
  setTask(time-.1, time, new DefaultTaskMap(posDiffTMT, world, endeffRef, NoVector, object, NoVector), sumOfSqrTT, NoArr, 1e3);
//  setTask(time-.1, time, new DefaultTaskMap(quatDiffTMT, world, endeffRef, NoVector, object, NoVector), sumOfSqrTT, NoArr, 1e3);

  //-- object needs to be static (this enforces q-states of new joints to adopt object pose)
  setTask(time-.1, time+.1, new DefaultTaskMap(posDiffTMT, world, object), sumOfSqrTT, NoArr, 1e3, 1);
  setTask(time-.1, time+.1, new DefaultTaskMap(quatDiffTMT, world, object), sumOfSqrTT, NoArr, 1e3, 1);
#endif
  //#    (EqualZero GJK Hand Obj){ time=[1 1] scale=100 } #touch is not necessary
//#    (MinSumOfSqr posDiff Hand Obj){ time=[.98 1] scale=1e3 }
//#    (MinSumOfSqr quatDiff Hand Obj){ time=[.98 1] scale=1e3 }

  setKinematicSwitch(time, true, "delete", object, NULL); //disconnect object from table
#if 0
  setKinematicSwitch(time, false, "rigidZero", graspRef, object); //connect graspRef with object
#else
  setKinematicSwitch(time, true, "freeZero", endeffRef, object); //connect graspRef with object
#endif
//#    (MakeJoint delete Obj){ time=1 }
//#    (MakeJoint rigidZero Hand Obj){ time=1 }

  if(stepsPerPhase>2){ //otherwise: no velocities
    setTask(time-.2, time-.05, new DefaultTaskMap(posTMT, world, object), sumOfSqrTT, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+.15, new DefaultTaskMap(posTMT, world, object), sumOfSqrTT, {0.,0.,.1}, 1e1, 1); // move up
//#    (MinSumOfSqr pos Obj){ order=1 scale=1e-1 time=[0 0.15] target=[0 0 .1] } # move up
  }
}

void KOMO::setPlace(double time, const char* endeffRef, const char* object, const char* placeRef, bool effKinMode){
  if(stepsPerPhase>2){ //otherwise: no velocities
    setTask(time-.15, time, new DefaultTaskMap(posTMT, world, object), sumOfSqrTT, {0.,0.,-.1}, 1e1, 1);
    setTask(time, time+.15, new DefaultTaskMap(posTMT, world, object), sumOfSqrTT, {0.,0.,.1}, 1e1, 1); // move up
  }
//  time += .5;

  setTask(time-.02, time, new DefaultTaskMap(posDiffTMT, world, object, NoVector, placeRef, NoVector), sumOfSqrTT, {0.,0.,.1}, 1e-1);
//#    (MinSumOfSqr posDiff Obj Onto){ time=[1 1] target=[0 0 .2] scale=1000 } #1/2 metre above the thing

  setTask(time-.02, time, new DefaultTaskMap(vecTMT, world, object, Vector_z), sumOfSqrTT, {0.,0.,1.}, 1e2);
//#    (MinSumOfSqr vec Obj){ time=[1 1] vec1=[0 0 1] target=[0 0 1] scale=100} #upright

  //-- object needs to be static (this enforces q-states of new joints to adopt object pose)
  setTask(time +1./stepsPerPhase, time +1./stepsPerPhase, new DefaultTaskMap(posDiffTMT, world, object), sumOfSqrTT, NoArr, 1e3, 1);
  setTask(time +1./stepsPerPhase, time +1./stepsPerPhase, new DefaultTaskMap(quatDiffTMT, world, object), sumOfSqrTT, NoArr, 1e3, 1);

#if 0
  mlr::String& graspRef = world.getShapeByName(endeffRef)->body->outLinks.last()->to->shapes.scalar()->name;
  setKinematicSwitch(time, true, "delete", graspRef, object); //disconnect object from grasp ref
#else
  setKinematicSwitch(time, false, "delete", endeffRef, object); //disconnect object from grasp ref
#endif

  //#    (MakeJoint delete Hand Obj){ time=1 }
  if(!effKinMode){
    setKinematicSwitch(time, false, "rigidAtTo", placeRef, object); //connect object to table
    //#    (MakeJoint rigidAtTo Onto Obj){ time=1 }
  }else{
//    setKinematicSwitch(time, "rigidZero", placeRef, object); //connect object to table
    ors::Transformation rel = 0;
    rel.addRelativeTranslation( 0., 0., .5*(height(world.getShapeByName(object)) + height(world.getShapeByName(placeRef))));
    setKinematicSwitch(time, false, "transXYPhiZero", placeRef, object, rel ); //connect object to table
    //#    (MakeJoint transXYPhiZero Onto Obj){ time=0 from=<T t(0 0 .3)> }
  }
}

void KOMO::setSlowAround(double time, double delta){
  if(stepsPerPhase>2) //otherwise: no velocities
    setTask(time-.02, time+.02, new TaskMap_qItself(), sumOfSqrTT, NoArr, 1e1, 1);
  //#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=1e1 } #slow down
}

void KOMO::setAbstractTask(double phase, const NodeL& facts, bool effKinMode){
  CHECK(phase<=maxPhase,"");
  for(Node *n:facts){
    if(!n->parents.N) continue;
    StringL symbols;
    for(Node *p:n->parents) symbols.append(&p->keys.last());
    if(n->keys.N && n->keys.last()=="komoGrasp"){
      setGrasp(phase, *symbols(0), *symbols(1), effKinMode);
    }
    if(n->keys.N && n->keys.last()=="komoPlace"){
      setPlace(phase, *symbols(0), *symbols(1), *symbols(2), effKinMode);
    }
  }
}

void KOMO::setAlign(double startTime, double endTime, const char* shape, const arr& whichAxis, const char* shapeRel, const arr& whichAxisRel, TermType type, const arr& target, double prec){
#if 0
  mlr::String map;
  map <<"map=vecAlign ref1="<<shape;
  if(whichAxis) map <<" vec1=[" <<whichAxis <<']';
  if(shapeRel) map <<" ref2=" <<shapeRel <<" vec2=" <<;
  if(whichAxisRel) map <<" vec2=[" <<whichAxisRel <<']';
  setTask(startTime, endTime, map, type, target, prec);
#else
  setTask(startTime, endTime, new DefaultTaskMap(vecAlignTMT, world, shape, ors::Vector(whichAxis), shapeRel, ors::Vector(whichAxisRel)), type, target, prec);
#endif

}

void KOMO::setCollisions(bool hardConstraint, double margin, double prec){
  if(hardConstraint){ //interpreted as hard constraint (default)
    setTask(0., -1., new CollisionConstraint(margin), ineqTT, NoArr, prec);
  }else{ //cost term
    setTask(0., -1., new ProxyTaskMap(allPTMT, {0u}, margin), sumOfSqrTT, NoArr, prec);
  }
}

//===========================================================================
//
// config
//


void KOMO::setConfigFromFile(){
//  Graph model;
//  FILE(mlr::getParameter<mlr::String>("KOMO/modelfile")) >>model;
  ors::KinematicWorld W(mlr::getParameter<mlr::String>("KOMO/modelfile"));
  setModel(
        W,
        mlr::getParameter<bool>("KOMO/meldFixedJoints", false),
        mlr::getParameter<bool>("KOMO/makeConvexHulls", true),
        mlr::getParameter<bool>("KOMO/makeSSBoxes", false),
        mlr::getParameter<bool>("KOMO/activateAllContact", false)
        );
  setTiming(
        mlr::getParameter<uint>("KOMO/phases"),
        mlr::getParameter<uint>("KOMO/stepsPerPhase"),
        mlr::getParameter<double>("KOMO/durationPerPhase", 5.),
        mlr::getParameter<uint>("KOMO/k_order", 2)
        );
}

void KOMO::setMoveTo(ors::KinematicWorld& world, ors::Shape& endeff, ors::Shape& target, byte whichAxesToAlign){
  if(MP) delete MP;
  MP = new MotionProblem(world);

  setTasks(*MP, endeff, target, whichAxesToAlign, 1, -1, -1.);
  reset();
}

void KOMO::setSpline(uint splineT){
  mlr::Spline S;
  S.setUniformNonperiodicBasis(MP->T, splineT, 2);
  uint n=MP->dim_x(0);
  splineB = zeros(S.basis.d0*n, S.basis.d1*n);
  for(uint i=0;i<S.basis.d0;i++) for(uint j=0;j<S.basis.d1;j++)
    splineB.setMatrixBlock(S.basis(i,j)*eye(n,n), i*n, j*n);
  z = pseudoInverse(splineB)* x;
}

void KOMO::reset(){
  x = MP->getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config
  if(splineB.N){
    z = pseudoInverse(splineB)* x;
  }
}

void KOMO::step(){
  NIY;
}

void KOMO::run(){
  ors::KinematicWorld::setJointStateCount=0;
  if(MP->T){
    if(!splineB.N)
      optConstrained(x, dual, Convert(*MP), OPT(verbose=2));
    else{
      arr a,b,c,d,e;
      ConstrainedProblem P0 = conv_KOrderMarkovFunction2ConstrainedProblem(*MP);
      P0(a,b,c,NoTermTypeA, x);
      ConstrainedProblem P = conv_linearlyReparameterize(P0, splineB);
      P(a,b,NoArr,NoTermTypeA,z);
      optConstrained(z, dual, P, OPT(verbose=2));
    }
  }else{
    HALT("deprecated")
    optConstrained(x, dual, MP->InvKinProblem(), OPT(verbose=2));
  }
  cout <<"** optimization time=" <<mlr::timerRead()
      <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
  MP->costReport(false);
}

Graph KOMO::getReport(){
  return MP->getReport();
}

void KOMO::checkGradients(){
  if(MP->T){
    if(!splineB.N)
      checkJacobianCP(Convert(*MP), x, 1e-4);
    else{
      ConstrainedProblem P0 = conv_KOrderMarkovFunction2ConstrainedProblem(*MP);
      ConstrainedProblem P = conv_linearlyReparameterize(P0, splineB);
      checkJacobianCP(P, z, 1e-4);
    }
  }else{
    checkJacobianCP(MP->InvKinProblem(), x, 1e-4);
  }
}

void KOMO::displayTrajectory(double delay){
#if 1
  MP->displayTrajectory(1, "KOMO planned trajectory", delay);
#else
  if(MP->T){
    ::displayTrajectory(x, 1, world, MP->switches, "KOMO planned trajectory", delay);
  //  orsDrawProxies=true;
  // for(uint t=0;t<x.d0;t++){
  //   MP->setState(x[t]);
  //   MP->world.gl().update(STRING("KOMO (time " <<std::setw(3) <<t <<'/' <<x.d0 <<')'));
  // }
  }else{
    world.setJointState(x);
    world.stepSwift();
    world.gl().watch("KOMO InvKin mode");
  }
  // if(wait) MP->world.gl().watch();
#endif
}

//===========================================================================

arr moveTo(ors::KinematicWorld& world,
           ors::Shape &endeff,
           ors::Shape& target,
           byte whichAxesToAlign,
           uint iterate,
           int timeSteps,
           double duration){

  MotionProblem MP(world);

  setTasks(MP, endeff, target, whichAxesToAlign, iterate, timeSteps, duration);


  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config

  //-- optimize
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  ors::KinematicWorld::setJointStateCount=0;
  for(uint k=0;k<iterate;k++){
    mlr::timerStart();
    if(colPrec<0){
      optConstrained(x, NoArr, Convert(MP), OPT(verbose=2)); //parameters are set in cfg!!
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    }else{
      optNewton(x, Convert(MP), OPT(verbose=2));
    }
    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
    //    checkJacobian(Convert(MF), x, 1e-5);
    MP.costReport();
  }

  return x;
}

void setTasks(MotionProblem& MP,
              ors::Shape &endeff,
              ors::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration){

  //-- parameters
  double posPrec = mlr::getParameter<double>("KOMO/moveTo/precision", 1e3);
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = mlr::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = mlr::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
  double alignPrec = mlr::getParameter<double>("KOMO/moveTo/alignPrecision", 1e3);

  //-- set up the MotionProblem
  target.cont=false; //turn off contact penalization with the target

  MP.world.swift().initActivations(MP.world);
  //MP.world.watch(false);

  if(timeSteps>=0) MP.setTiming(timeSteps, duration);
  if(timeSteps==0) MP.k_order=1;

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(MP.world), sumOfSqrTT);
  if(timeSteps!=0){
    t->map.order=2; //make this an acceleration task!
  }else{
    t->map.order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  if(timeSteps!=0){
    t = MP.addTask("final_vel", new TaskMap_qItself(), sumOfSqrTT);
    t->map.order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T, {0.}, zeroVelPrec);
  }

  if(colPrec<0){ //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin), ineqTT);
    t->setCostSpecs(0, MP.T, {0.}, 1.);
  }else{ //cost term
    t = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0u}, margin), sumOfSqrTT);
    t->setCostSpecs(0, MP.T, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector), sumOfSqrTT);
  t->setCostSpecs(MP.T, MP.T, {0.}, posPrec);


  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    ors::Vector axis;
    axis.setZero();
    axis(i)=1.;
    t = MP.addTask(STRING("endeff_align_"<<i),
                   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis),
                   sumOfSqrTT);
    t->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
  }
}

//===========================================================================



