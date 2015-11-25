#include "actionMachine.h"
#include "actionMachine_internal.h"

//#include <Hardware/gamepad/gamepad.h>
#include <Motion/pr2_heuristics.h>
#include <FOL/fol.h>
#include <pr2/rosalvar.h>

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

ActionMachine::ActionMachine():Module("ActionMachine"), initStateFromRos(false){
  ActionL::memMove=true;
  Kp = ARR(1.);
  Kd = ARR(1.);
  s = new sActionMachine();
  world = &s->world;
}

ActionMachine::~ActionMachine(){
  delete s;
}

void ActionMachine::open(){
  s->world.getJointState(s->q, s->qdot);

  s->feedbackController.H_rate_diag = mlr::getParameter<double>("Hrate", 1.)*pr2_reasonable_W(s->world);
  s->feedbackController.qitselfPD.y_ref = s->q;
  s->feedbackController.qitselfPD.setGains(.0,10.);

  { // read machine.fol
    mlr::FileToken fil("machine.fol");
    if(fil.exists()){
      KB.writeAccess();
      fil >> KB();
      KB().checkConsistency();
      KB()>> FILE("z.initialKB");
      KB.deAccess();
    }else{
      KB.writeAccess();
      new Node_typed<bool>(KB(), {"conv"}, {}, NULL, false);
      new Node_typed<bool>(KB(), {"contact"}, {}, NULL, false);
      new Node_typed<bool>(KB(), {"timeout"}, {}, NULL, false);
//new Node_typed<bool>(KB(), {"CoreTasks"}, {}, NULL, false);
//new Node_typed<bool>(KB(), {"moving"}, {}, NULL, false);
      new Node_typed<Graph>(KB(), {"STATE"}, {}, new Graph(), true);
      KB().checkConsistency();
      KB()>> FILE("z.initialKB");
      KB.deAccess();
    }
  }

  KB.readAccess();
  Node *tasks = KB()["Tasks"];
  if(tasks){
    parseTaskDescriptions(tasks->graph());
  }
  KB.deAccess();

  mlr::open(fil,"z.actionMachine");

  bool useRos = mlr::getParameter<bool>("useRos",false);
  if(useRos){
    initStateFromRos=true;
  }
}

void ActionMachine::step(){
  static uint t=0;
  t++;

  if(initStateFromRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;

    ctrl_obs.waitForNextRevision();
    cout <<"REMOTE joint dimension=" <<ctrl_obs.get()->q.N <<endl;
    cout <<"LOCAL  joint dimension=" <<s->feedbackController.world.q.N <<endl;

    if(ctrl_obs.get()->q.N==s->feedbackController.world.q.N
       && ctrl_obs.get()->qdot.N==s->feedbackController.world.q.N){ //all is good
      //-- set current state
      s->q = ctrl_obs.get()->q;
      s->qdot = ctrl_obs.get()->qdot;
      s->feedbackController.setState(s->q, s->qdot);
      cout <<"** GO!" <<endl;
      initStateFromRos = false;
    }else{
      if(t>20){
        HALT("sync'ing real PR2 with simulated failed - using useRos=false")
      }
    }
  }

  if(!(t%5))
    s->feedbackController.world.watch(false, STRING("local operational space controller state t="<<(double)t/100.));

  // Sync alvar marker
  AlvarMarkers markers = ar_pose_marker.get();
  syncMarkers(s->feedbackController.world, markers);

  //-- do the logic of transitioning between actions, stopping/sequencing them, querying their state
//  transition();
  transitionFOL( .01*t, true);// (t<=1) );

  //-- set gains to default value (can be overwritten by other actions)
  Kp=ARR(1.);
  Kd=ARR(1.);

  //-- check the gamepad
  arr gamepad = gamepadState.get();
  if(stopButtons(gamepad)){
    cout <<"STOP" <<endl;
    KB.writeAccess();
    Node *quitSymbol = KB()["quit"];
    KB().getNode("STATE")->graph().append<bool>({},{quitSymbol}, NULL, false);
    KB.deAccess();
//    moduleShutdown().incrementValue();
  }

  //-- get access to the list of actions
  A.writeAccess();

  //  cout <<"** active actions:";
//  reportActions(A());

  // report on force:
//  arr fL = wrenchL.get()();
//  fil <<"wrenchL=" <<fL <<' ' <<length(fL) <<endl;


  //-- code to output force signals
  if(true){
    ors::Shape *ftL_shape = world->getShapeByName("endeffForceL");
    arr fLobs = ctrl_obs.get()->fL;
    arr uobs =  ctrl_obs.get()->u_bias;
//    cout <<fLobs <<endl;
    if(fLobs.N && uobs.N){
      arr Jft, J;
      world->kinematicsPos(NoArr, J, ftL_shape->body, ftL_shape->rel.pos);
      world->kinematicsPos_wrtFrame(NoArr, Jft, ftL_shape->body, ftL_shape->rel.pos, world->getShapeByName("l_ft_sensor"));
      Jft = inverse_SymPosDef(Jft*~Jft)*Jft;
      J = inverse_SymPosDef(J*~J)*J;
      mlr::arrayBrackets="  ";
      fil <<t <<' ' <<zeros(3) <<' ' << Jft*fLobs << " " << J*uobs << endl;
      mlr::arrayBrackets="[]";
    }
  }

  //-- call the step method for each action
  for(Action *a : A()) {
    if(a->active){
      a->step(*this);
      for(CtrlTask *t:a->tasks) t->active=true;
      a->actionTime += .01;
    } else {
      for(CtrlTask *t:a->tasks) t->active=false;
    }
  }

  //-- compute the feedback controller step and iterate to compute a forward reference
  //first collect all tasks of all actions into the feedback controller:
  s->feedbackController.tasks.clear();
  for(Action *a : A()) for(CtrlTask *t:a->tasks) s->feedbackController.tasks.append(t);
  //now operational space control
  for(uint tt=0;tt<10;tt++){
    arr a = s->feedbackController.operationalSpaceControl();
    s->q += .001*s->qdot;
    s->qdot += .001*a;
    s->feedbackController.setState(s->q, s->qdot);
  }

  //-- first zero references
  s->refs.Kp = ARR(1.); //Kp;
  s->refs.Kd = ARR(1.); //Kd;
  s->refs.fL = zeros(6);
  s->refs.fR = zeros(6);
  s->refs.Ki.clear();
  s->refs.J_ft_inv.clear();
  s->refs.u_bias = zeros(s->q.N);

  //-- compute the force feedback control coefficients
  uint count=0;
  for(Action *a : A()) {
    if(a->active && a->tasks.N && a->tasks(0)->f_ref.N){
      count++;
      if(count!=1) HALT("you have multiple active force control tasks - NIY");
      a->tasks(0)->getForceControlCoeffs(s->refs.fL, s->refs.u_bias, s->refs.Ki, s->refs.J_ft_inv, *world);
    }
  }
  if(count==1) s->refs.Kp = .5;


  // s->MP.reportCurrentState();
  A.deAccess();

  //-- send the computed movement to the robot
  s->refs.q =  s->q;
  s->refs.qdot = zeros(s->q.N);
  s->refs.gamma = 1.;
  ctrl_ref.set() = s->refs;
}

void ActionMachine::close(){
  fil.close();
}

void ActionMachine::parseTaskDescription(Graph& td){
  Node *t = td.isNodeOfParentGraph;
  mlr::String type=td["type"]->V<mlr::String>();
  if(type=="homing"){
    new Homing(*this, t->parents(0)->keys.last());
  }else if(type=="forceCtrl"){
    new PushForce(*this, td["ref1"]->V<mlr::String>(), td["target"]->V<arr>(), td["timeOut"]->V<double>());
  }else{
    DefaultTaskMap *map = new DefaultTaskMap(td, *world);
    CtrlTask* task = new CtrlTask(t->parents(0)->keys.last(), *map, td);
    task->active=false;
    new FollowReference(*this, t->parents(0)->keys.last(), task);
  }
}

void ActionMachine::parseTaskDescriptions(const Graph& tds){
  cout <<"Instantiating task descriptions:\n" <<tds <<endl;
  for(Node *t:tds) parseTaskDescription(t->graph());
}

void ActionMachine::removeAction(Action* a, bool hasLock){
  if(!hasLock) A.set()->removeValue(a);
  else A().removeValue(a);
  delete a;
}


void ActionMachine::transitionFOL(double time, bool forceChaining){
  bool changes=false;
  KB.writeAccess();
  KB().checkConsistency();
  //-- check new successes and fails and add to symbolic state
  Node* convSymbol = KB().getNode("conv");  CHECK(convSymbol,"");
  Node* contactSymbol = KB().getNode("contact");  CHECK(contactSymbol,"");
  Node* timeoutSymbol = KB().getNode("timeout");  CHECK(timeoutSymbol,"");
  Graph& state = KB().getNode("STATE")->graph();
  cout <<"STATE = " <<state <<endl;
  A.readAccess();
  for(Action *a:A()) if(a->active){
    if(a->finishedSuccess(*this)){
      Node *newit = state.append<bool>({}, {a->symbol, convSymbol}, new bool(true), true);
      if(getEqualFactInKB(state, newit)) delete newit;
      else changes=true;
    }
    if(a->indicateTimeout(*this)){
      Node *newit = state.append<bool>({}, {a->symbol, timeoutSymbol}, new bool(true), true);
      if(getEqualFactInKB(state, newit)) delete newit;
      else changes=true;
    }
  }
  if(getContactForce()>5.){
    Node *newit = state.append<bool>({}, {contactSymbol}, new bool(true), true);
    if(getEqualFactInKB(state, newit)) delete newit;
    else changes=true;
  }
  A.deAccess();
  KB().checkConsistency();

  if(changes || forceChaining){
    cout <<"STATE (changed by real world at t=" <<time <<"):"; state.write(cout, " "); cout <<endl;
    forwardChaining_FOL(KB(), NULL, NoGraph, 0);
//    state = getLiteralsOfScope(KB());
    cout <<"STATE (transitioned by FOL   at t=" <<time <<"):"; state.write(cout, " "); cout <<endl;

    A.writeAccess();
    for(Action *a:A()){
      if(!a->symbol){ a->active=false;  continue; }
      bool act=false;
      for(Node *lit:a->symbol->parentOf){
        if(&lit->container==&state && lit->keys.N==0 && lit->parents.N>0){ act=true; break; }
      }
      if(act) a->active=true;
      else a->active=false;
    }
    A.deAccess();
  }
  KB.deAccess();
}

double ActionMachine::getContactForce(){
  arr fL = ctrl_obs.get()->fL;
  return length(fL);
}

void ActionMachine::waitForActionCompletion(Action* a){
  for(bool cont=true;cont;){
    A.waitForNextRevision();
    A.readAccess();
    if(!A().contains(a)) cont=false;
    A.deAccess();
  }
}

void ActionMachine::waitForActionCompletion() {
  bool cont = true;
  while (cont) {
    A.waitForNextRevision();
    A.readAccess();
    if (A().N == 0 || (A().N == 1 && A()(0)->name == "CoreTasks")) {
      cont=false;
    }
    A.deAccess();
  }
}

void ActionMachine::waitForQuitSymbol() {
  bool cont = true;
  while (cont) {
    KB.waitForNextRevision();
    KB.readAccess();
    Node* quitSymbol = KB().getNode("quit");
    if(!quitSymbol){
      MLR_MSG("WARNING: no quit symbol!");
      return;
    }
    for(Node *f:quitSymbol->parentOf){
      if(f->container.isNodeOfParentGraph && f->container.isNodeOfParentGraph->keys.N && f->container.isNodeOfParentGraph->keys(0)=="STATE" && f->parents.N==1){ cont=false; break; }
    }
    KB.deAccess();
  }
}

//===========================================================================
// Helper functions
//

void reportActions(ActionL &A){
  cout <<"** REPORT ON CURRENT STATE OF ACTIONS" <<endl;
  for (Action *a : A) a->reportState(cout);
}
