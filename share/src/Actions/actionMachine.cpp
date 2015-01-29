#include "actions.h"
#include "actionMachine_internal.h"

#include <Hardware/gamepad/gamepad.h>
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>

//===========================================================================
//Singleton<SymbolL> symbols;

const char* ActionStateString[7] = {
 "trueLV", "falseLV", "inactive", "queued", "active", "failed", "success"
};

const char* getActionStateString(ActionState actionState){ return ActionStateString[actionState]; }

//===========================================================================
//
// ActionMachine
//

ActionMachine::ActionMachine():Module("ActionMachine"){
  ActionL::memMove=true;
  s = new sActionMachine();
}

ActionMachine::~ActionMachine(){
  delete s;
}

void ActionMachine::open(){
  s->world.getJointState(s->q, s->qdot);

  s->feedbackController.H_rate_diag = pr2_reasonable_W(s->world);
  s->feedbackController.qitselfPD.y_ref = s->q;
//  s->MP.qitselfPD.setGains(1.,10.);

  if(MT::getParameter<bool>("useRos",false)){
	//-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      ctrl_obs.var->waitForNextRevision();
      if(ctrl_obs.get()->q.N==s->feedbackController.world.q.N
         && ctrl_obs.get()->qdot.N==s->feedbackController.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    s->q = ctrl_obs.get()->q;
    s->qdot = ctrl_obs.get()->qdot;
    s->feedbackController.setState(s->q, s->qdot);
  }
  //arr fL_base = S.fL_obs.get();
}

void ActionMachine::step(){
  static uint t=0;
  t++;

  if(!(t%10))
    s->feedbackController.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

  //-- do the logic of transitioning between actions, stopping/sequencing them, querying their state
  transition();

  //defaults
  s->refs.fR = ARR(0., 0., 0.);
  s->refs.Kq_gainFactor = ARR(1.);

  //-- check the gamepad
  arr gamepad = gamepadState.get();
  if(stopButtons(gamepad)) engine().shutdown.incrementValue();

  //-- get access to the list of actions
  A.writeAccess();

  //  cout <<"** active actions:";
  reportActions(A());

  //-- call the step method for each action
  for(GroundedAction *a : A()) {
    if(a->actionState==ActionState::active){
      a->step(*this);
      for(PDtask *t:a->tasks) t->active=true;
      a->actionTime += .01;
    } else {
      for(PDtask *t:a->tasks) t->active=false;
    }
  }

  //-- compute the feedback controller step
  //first collect all tasks of all actions into the feedback controller:
  s->feedbackController.tasks.clear();
  for(GroundedAction *a : A()) for(PDtask *t:a->tasks) s->feedbackController.tasks.append(t);
  //now operational space control
  for(uint tt=0;tt<10;tt++){
    arr a = s->feedbackController.operationalSpaceControl();
    s->q += .001*s->qdot;
    s->qdot += .001*a;
    s->feedbackController.setState(s->q, s->qdot);
  }

  // s->MP.reportCurrentState();
  A.deAccess();

  //-- send the computed movement to the robot
  s->refs.q=s->q;
  s->refs.qdot = zeros(s->q.N);
  s->refs.u_bias = zeros(s->q.N);
  s->refs.gamma = 1.;
  ctrl_ref.set() = s->refs;
}

void ActionMachine::close(){
}

void ActionMachine::add_sequence(GroundedAction *action1,
                                 GroundedAction *action2,
                                 GroundedAction *action3,
                                 GroundedAction *action4)
{
  action2->actionState = ActionState::queued;
  action2->dependsOnCompletion.append(action1);
  if (action3) {
    action3->actionState = ActionState::queued;
    action3->dependsOnCompletion.append(action2);
  }
  if (action4) {
    action4->actionState = ActionState::queued;
    action4->dependsOnCompletion.append(action3);
  }
}

void ActionMachine::removeGroundedAction(GroundedAction* a, bool hasLock){
  if(!hasLock) A.set()->removeValue(a);
  else A().removeValue(a);
  delete a;
}

void ActionMachine::transition(){
  A.writeAccess();

  //-- first remove all old successes and fails
  for_list_rev(GroundedAction, a, A()) if(a->actionState==ActionState::success || a->actionState==ActionState::failed){
    removeGroundedAction(a, true);
    a=NULL; //a has deleted itself, for_list_rev should be save, using a_COUNTER
  }

  //-- check new successes and fails
  for(GroundedAction *a:A()) if(a->actionState==ActionState::active){
    if(a->finishedSuccess(*this)) a->actionState=ActionState::success;
    if(a->finishedFail(*this)) a->actionState=ActionState::failed;
  }

  //-- progress with queued
  for(GroundedAction *a:A()) if(a->actionState==ActionState::queued){
    bool fail=false, succ=true;
    for(GroundedAction *b:a->dependsOnCompletion){
      if(b->actionState==ActionState::failed) fail=true;
      if(b->actionState!=ActionState::success) succ=false;
    }
    if(fail) a->actionState=ActionState::failed; //if ONE dependence failed -> fail
    if(succ) a->actionState=ActionState::active; //if ALL dependences succ -> active
    //in all other cases -> queued
  }

  A.deAccess();
}

void ActionMachine::waitForActionCompletion(GroundedAction* a){
  for(bool cont=true;cont;){
    A.var->waitForNextRevision();
    A.readAccess();
    if(!A().contains(a)) cont=false;
    A.deAccess();
  }
}

void ActionMachine::waitForActionCompletion() {
  bool cont = true;
  while (cont) {
    A.var->waitForNextRevision();
    A.readAccess();
    if (A().N == 0 || (A().N == 1 && A()(0)->name == "CoreTasks")) {
      cont=false;
    }
    A.deAccess();
  }
}
//===========================================================================
// GroundedAction
//

GroundedAction::GroundedAction(ActionMachine& actionMachine, const char* name, ActionState actionState)
  : name(name), actionState(actionState), actionTime(0.){
  actionMachine.A.set()->append(this);
}

GroundedAction::~GroundedAction(){
//  for (PDtask *t : tasks) actionMachine.s->feedbackController.tasks.removeValue(t);
  listDelete(tasks);
}

void GroundedAction::reportState(ostream& os){
  os <<"Action '" <<name
    <<"':  actionState=" << getActionStateString(actionState)
    <<"  actionTime=" << actionTime
    <<"  PDtasks:" <<endl;
  for(PDtask* t: tasks) t->reportState(os);
  reportDetails(os);
}

//===========================================================================
// Helper functions
//

void reportActions(ActionL &A){
  cout <<"** REPORT ON CURRENT STATE OF ACTIONS" <<endl;
  for (GroundedAction *a : A) a->reportState(cout);
}
