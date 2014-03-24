
#include "actionMachine_internal.h"

//===========================================================================

Singleton<SymbolL> symbols;

const char *GroundActionValueString[] = { "trueLV", "falseLV", "inactive", "queued", "active", "failed", "success" };

//===========================================================================

ActionMachine::ActionMachine():Module("ActionMachine"), ros(NULL){
  ActionL::memMove=true;
  s = new sActionMachine();
}

ActionMachine::~ActionMachine(){
  delete s;
}

void ActionMachine::open(){
  s->world.getJointState(s->q, s->qdot);

  s->MP.H_rate_diag = pr2_reasonable_W(s->world);
  s->MP.qitselfPD.y_ref = s->q;
  s->MP.qitselfPD.v_ref.resizeAs(s->q).setZero();
#if 0
  s->MP.nullSpacePD.active=false;
#else
  s->MP.qitselfPD.setGains(0.,100.);
  s->MP.qitselfPD.prec=100.;
#endif

  //-- wait for first q observation!
  if(ros){
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      ctrl_obs.var->waitForNextRevision();
      if(ctrl_obs.get()->q.N==s->MP.world.q.N
         && ctrl_obs.get()->qdot.N==s->MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    s->q = ctrl_obs.get()->q;
    s->qdot = ctrl_obs.get()->qdot;
    s->MP.setState(s->q, s->qdot);
  }
  s->zero_qdot.resize(s->qdot.N).setZero();
  //arr fL_base = S.fL_obs.get();
}

void ActionMachine::step(){
  static uint t=0;
  t++;

  transition();

  //defaults
  s->refs.fR = ARR(0., 0., 0.);
  s->refs.fR_gainFactor = 0.;
  s->refs.Kp_gainFactor = 1.;

  arr joypadState = joystickState.get();
  if(stopButtons(joypadState)) engine().shutdown.incrementValue();

  //-- gain access
  A.readAccess();

//  cout <<"** active actions:";
  reportActions(A());
  for(GroundedAction *a : A()) if(a->value==GroundedAction::active){
//    cout <<' ' <<a->symbol.name;
    for(PDtask *t:a->tasks) t->active=true;
    if(a->symbol==moveEffTo){
      //nothing to be done
    }
    if(a->symbol==moveEffTo){
      //dynamic_cast<MoveEffTo_ActionSymbol&>(a->symbol).task->y_ref = a->poseArg1;
    }
    if(a->symbol==pushForce){
      cout <<"FORCE TASK" <<endl;
      s->refs.fR = a->poseArg1;
      s->refs.fR_gainFactor = 1.;
      s->refs.Kp_gainFactor = .2;
    }
  } else {
    for(PDtask *t:a->tasks) t->active=false;
  }

  cout <<"FL=" <<ctrl_obs.get()->fL <<endl;

  for(uint tt=0;tt<10;tt++){
    arr a = s->MP.operationalSpaceControl();
    s->q += .001*s->qdot;
    s->qdot += .001*a;
  }
  s->MP.setState(s->q, s->qdot);
  if(!(t%10))
    s->MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

  A.deAccess();

  s->refs.q=s->q;
  s->refs.qdot=s->zero_qdot;
  ctrl_ref.set() = s->refs;
  if(ros) ros->publishJointReference();
}

void ActionMachine::close(){
}

GroundedAction* ActionMachine::addGroundedAction(ActionSymbol &sym,
                               const char *shapeArg1, const char *shapeArg2,
                               const arr& poseArg1, const arr &poseArg2){
  GroundedAction *a = new GroundedAction(sym);
  if(shapeArg1) a->shapeArg1 = shapeArg1;
  if(shapeArg2) a->shapeArg2 = shapeArg2;
  if(&poseArg1) a->poseArg1 = poseArg1;
  if(&poseArg2) a->poseArg2 = poseArg2;
  a->symbol.initYourself(*a, *this);
  a->value=GroundedAction::active; //TODO!

  A.set()->append(a);
  return a;
}

void ActionMachine::removeGroundedAction(GroundedAction* a, bool hasLock){
  a->symbol.deinitYourself(*a, *this);
  if(!hasLock) A.set()->removeValue(a);
  else A().removeValue(a);
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
    cout <<a->symbol.name <<" {" <<a->shapeArg1 <<' ' <<a->shapeArg2 <<" } { " <<a->poseArg1 <<' ' <<a->poseArg2 <<" }:";
    cout <<"value=" <<GroundActionValueString[a->value] <<endl;
  }
}

void ActionMachine::transition(){
  Access_typed<ActionL>::WriteToken lock(&A); //kills itself
  //const auto& lock=A.set();

  //-- first remove all old successes and fails
  for_list_rev(GroundedAction, a, A()) if(a->value==GroundedAction::success || a->value==GroundedAction::failed){
    removeGroundedAction(a, true);
    a=NULL; //a has deleted itself, for_list_rev should be save, using a_COUNTER
  }

  //-- check new successes and fails
  for(GroundedAction *a:A()) if(a->value==GroundedAction::active){
    if(a->symbol.finishedSuccess(*a, *this)) a->value=GroundedAction::success;
    if(a->symbol.finishedFail(*a, *this)) a->value=GroundedAction::failed;
  }

  //-- progress with queued
  for(GroundedAction *a:A()) if(a->value==GroundedAction::queued){
    bool fail=false, succ=true;
    for(GroundedAction *b:a->dependsOnCompletion){
      if(b->value==GroundedAction::failed) fail=true;
      if(b->value!=GroundedAction::success) succ=false;
    }
    if(fail) a->value=GroundedAction::failed; //if ONE dependence failed -> fail
    if(succ) a->value=GroundedAction::active; //if ALL dependences succ -> active
    //in all other cases -> queued
  }
}

void ActionMachine::waitForActionCompletion(GroundedAction* a){
  for(bool cont=true;cont;){
    A.var->waitForNextRevision();
    A.readAccess();
    if(!A().contains(a)) cont=false;
    A.deAccess();
  }

}
