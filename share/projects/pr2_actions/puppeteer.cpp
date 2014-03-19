#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>

#include "puppeteer.h"
#include <System/ros/roscom.h>

Symbol moveEffTo = {0, MT::String("MoveEffTo"), 2};
Symbol coreTasks = {1, MT::String("CoreTasks"), 0};
Symbol alignEffTo = {2, MT::String("AlignEffTo"), 2};
Symbol pushForce = {3, MT::String("PushForce"), 2};

struct PuppeteerSystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, joystickState);
  RosCom *ros;
  PuppeteerSystem():ros(NULL){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false))
      ros = addModule<RosCom>(NULL, Module_Thread::loopWithBeat, .001);
    connect();
  }
};

struct sPuppeteer{
  ors::KinematicWorld world;
  FeedbackMotionControl MP;
//  Gamepad2Tasks j2t;
  PuppeteerSystem S;
  arr q, qdot, zero_qdot;
  CtrlMsg refs;
  sPuppeteer():world("model.kvg"), MP(world,false)/*, j2t(MP)*/{}
  void open();
  void step(uint t);
};

void sPuppeteer::open(){
  world.getJointState(q, qdot);

  MP.nullSpacePD.y_ref = q;
  MP.nullSpacePD.active=false;
  MP.H_rate_diag = pr2_reasonable_W(world);

  engine().open(S);

  //-- wait for first q observation!
  if(S.ros){
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      S.ctrl_obs.var->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==MP.world.q.N
         && S.ctrl_obs.get()->qdot.N==MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;
    MP.setState(q, qdot);
  }
  zero_qdot.resize(qdot.N).setZero();
  //arr fL_base = S.fL_obs.get();
}

void sPuppeteer::step(uint t){

  cout <<S.ctrl_obs.get()->fL <<endl;

  for(uint tt=0;tt<10;tt++){
    arr a = MP.operationalSpaceControl();
    q += .001*qdot;
    qdot += .001*a;
  }
  MP.setState(q, qdot);
  if(!(t%10))
    MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

  refs.q=q;
  refs.qdot=zero_qdot;
  S.ctrl_ref.set() = refs;
  if(S.ros) S.ros->publishJointReference();
}

void Puppeteer::open(){ s->open(); }

ATom* Puppeteer::addLiteral(const Symbol& sym,
                               const char *shapeArg1, const char *shapeArg2,
                               const arr& poseArg1, const arr &poseArg2){
  ATom *a = A.append(new ATom);
  a->symbol = sym;
  if(shapeArg1) a->shapeArg1 = shapeArg1;
  if(shapeArg2) a->shapeArg2 = shapeArg2;
  if(&poseArg1) a->poseArg1 = poseArg1;
  if(&poseArg2) a->poseArg2 = poseArg2;

  if(a->symbol==moveEffTo){
    PDtask *task = s->MP.addPDTask(STRING("MoveEffTo_" <<a->shapeArg1), .1, .8, posTMT, a->shapeArg1);
    a->tasks.append(task);
  }
  if(a->symbol==alignEffTo){
    PDtask *task = s->MP.addPDTask(STRING("AlignEffTo_" <<a->shapeArg1), .2, .8, vecTMT, a->shapeArg1, ors::Vector(a->poseArg1));
    task->y_ref = a->poseArg2;
    a->tasks.append(task);
  }
  if(a->symbol==pushForce){
    //nothing to be done?
  }
  if(a->symbol==coreTasks){
    PDtask *qitself = s->MP.addPDTask("DampMotion_qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, s->MP.H_rate_diag);
    qitself->setGains(0.,100.);
    qitself->y_ref = s->MP.nullSpacePD.y_ref;
    qitself->v_ref.setZero();
    qitself->prec=100.;
    a->tasks.append(qitself);

    PDtask *limits = s->MP.addPDTask("limits", .02, .8, qLimitsTMT);
    limits->active=true;
    limits->v_ref.setZero();
    limits->v_ref.setZero();
    limits->prec=100.;
    a->tasks.append(limits);
  }
  return a;
}

void Puppeteer::removeLiteral(ATom* a){
  for(PDtask *t:a->tasks) s->MP.tasks.removeValue(t);
  listDelete(a->tasks);
  A.removeValue(a);
}

Puppeteer::Puppeteer(){
  AtomL::memMove=true;
  s = new sPuppeteer();
}
Puppeteer::~Puppeteer(){
  delete s;
}

void Puppeteer::run(double secs){
  //defaults
  s->refs.fR = ARR(0., 0., 0.);
  s->refs.fR_gainFactor = 0.;
  s->refs.Kp_gainFactor = 1.;

  for(uint t=0;;t++){
    //s->S.joystickState.var->waitForNextRevision();
    arr joypadState = s->S.joystickState.get();
    if(stopButtons(joypadState)) engine().shutdown.incrementValue();

    cout <<"active actions:";
    for(ATom *a : A) if(a->active){
      cout <<' ' <<a->symbol.name;
      if(a->symbol==moveEffTo){
        //nothing to be done
      }
      if(a->symbol==moveEffTo){
        a->tasks(0)->y_ref = a->poseArg1;
      }
      if(a->symbol==pushForce){
        cout <<"FORCE TASK" <<endl;
        s->refs.fR = a->poseArg1;
        s->refs.fR_gainFactor = 1.;
        s->refs.Kp_gainFactor = .2;
      }
    }

    //    q    = S.q_obs.get();
    //    qdot = S.qdot_obs.get();
    //    MP.setState(q,qdot);

    s->step(t);

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;

    if(((double)t)/100. > secs) break;
  }
}

void Puppeteer::close(){
  engine().close(s->S);
}

