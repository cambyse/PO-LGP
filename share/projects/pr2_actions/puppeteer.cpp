
#include "puppeteer_internal.h"

//===========================================================================

Singleton<SymbolL> symbols;

//===========================================================================

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

GroundedAction* Puppeteer::addGroundedAction(ActionSymbol &sym,
                               const char *shapeArg1, const char *shapeArg2,
                               const arr& poseArg1, const arr &poseArg2){
  GroundedAction *a = A.append(new GroundedAction(sym));
  if(shapeArg1) a->shapeArg1 = shapeArg1;
  if(shapeArg2) a->shapeArg2 = shapeArg2;
  if(&poseArg1) a->poseArg1 = poseArg1;
  if(&poseArg2) a->poseArg2 = poseArg2;

  a->symbol.initYourself(*a, *this);
//  if(a->symbol==alignEffTo){
//  }
//  if(a->symbol==pushForce){
//    //nothing to be done?
//  }
//  if(a->symbol==coreTasks){
//  }
  return a;
}

void Puppeteer::removeGroundedAction(GroundedAction* a){
  a->symbol.deinitYourself(*a, *this);
  A.removeValue(a);
}

Puppeteer::Puppeteer(){
  ActionL::memMove=true;
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
    for(GroundedAction *a : A) if(a->active){
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

//===========================================================================

void reportExistingSymbols(){
  for(Symbol *s:symbols()){
    cout <<"Symbol '" <<s->name <<"' nargs=" <<s->nargs;
    if(dynamic_cast<ActionSymbol*>(s)) cout <<" is ActionSymbol";
    cout <<endl;
  }
}

void reportActions(ActionL &A){
  cout <<"* ActionL" <<endl;
  for(GroundedAction *a:A){
    cout <<a->symbol.name <<" [" <<a->shapeArg1 <<' ' <<a->shapeArg2 <<" ] [ " <<a->poseArg1 <<' ' <<a->poseArg2 <<" ]" <<endl;
  }
}

void Puppeteer::transition(){
  //-- first remove all old successes and fails
  for_list_rev(GroundedAction, a, A) if(a->value==GroundedAction::success || a->value==GroundedAction::failed){
    removeGroundedAction(a);
    a=NULL; //a has deleted itself, for_list_rev should be save, using a_COUNTER
  }

  //-- check new successes and fails
  for(GroundedAction *a:A) if(a->value==GroundedAction::active){
    if(a->symbol.finishedSuccess(*a, *this)) a->value=GroundedAction::success;
    if(a->symbol.finishedFail(*a, *this)) a->value=GroundedAction::failed;
  }

  //-- progress with queued
  for(GroundedAction *a:A) if(a->value==GroundedAction::queued){
    bool fail=false, succ=true;
    for(GroundedAction *b:a->dependsOnCompletion){
      if(b->value==GroundedAction::failed) fail=true;
      if(!b->value==GroundedAction::success) succ=false;
    }
    if(fail) a->value=GroundedAction::failed; //if ONE dependence failed -> fail
    if(succ) a->value=GroundedAction::active; //if ALL dependences succ -> active
    //in all other cases -> queued
  }
}
