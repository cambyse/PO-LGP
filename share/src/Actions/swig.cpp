#include "swig.h"

#include <FOL/fol.h>
#include <Ors/ors.h>
#include "TaskControllerModule.h"
#include "RelationalMachineModule.h"
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <pr2/rosalvar.h>
#include <pr2/rosmacro.h>

ROSSUB("/robot_pose_ekf/odom_combined", geometry_msgs::PoseWithCovarianceStamped , pr2_odom)

// ============================================================================
struct SwigSystem : System{
  ACCESS(bool, quitSignal)
  ACCESS(bool, fixBase)
  ACCESS(RelationalMachine, RM)
  ACCESS(MT::String, effects)
  ACCESS(MT::String, state)
  ACCESS(ors::KinematicWorld, modelWorld)
  ACCESS(AlvarMarker, ar_pose_markers)

  Log _log;
  TaskControllerModule tcm;

  SwigSystem():_log("SwigSystem"), tcm(*this){
//    tcm = addModule<TaskControllerModule>(NULL, Module::loopWithBeat, .01);
    modelWorld.linkToVariable(tcm.modelWorld.v);  //tcm already created a variable -> you need to adopt THAT one in the system

    addModule<ActivitySpinnerModule>(NULL, Module::loopWithBeat, .01);
    addModule<RelationalMachineModule>(NULL, Module::listenFirst, .01);

    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
      addModule<ROSSUB_ar_pose_marker>(NULL, Module::loopWithBeat, 0.05);
      addModule<ROSSUB_pr2_odom>(NULL, Module::loopWithBeat, 0.02);
      // addModule<RosCom_ForceSensorSync>(NULL, Module::loopWithBeat, 1.);
    }
    connect();
    // make the base movable by default
    fixBase.set() = MT::getParameter<bool>("fixBase", false);
  }
};


// ============================================================================
MT::String lits2str(const stringV& literals, const dict& parameters=dict()){
  MT::String str;
  str <<'(';
  for(auto& i:literals) str <<' ' <<i;
  str <<')';
  if(parameters.size()){
    str <<'{';
    for(auto& p:parameters) str <<' ' <<p.first <<'=' <<p.second;
    str <<'}';
  }
  return str;
}

// ============================================================================
// ActionSwigInterface
ActionSwigInterface::ActionSwigInterface(): S(new SwigSystem){
  S->tcm.verbose=false;
  engine().open(*S, true);

  createNewSymbol("conv");
  createNewSymbol("contact");
  createNewSymbol("timeout");
  createNewSymbol("go");
//  new CoreTasks(*s->activity.machine);

  S->LOG(1) <<"Registered Activities=" <<activityRegistry();
  for(Node *n:activityRegistry()){
    S->LOG(1) <<"adding symbol for " <<n->keys(0);
    createNewSymbol(n->keys(0).p);
  }
  S->LOG(1) <<"Shape Symbols:";
  S->modelWorld.writeAccess();
  for(ors::Shape *sh:S->modelWorld().shapes){
    S->LOG(1) <<"adding symbol for Shape " <<sh->name;
    createNewSymbol(sh->name.p);
  }
  S->modelWorld.deAccess();
}


ActionSwigInterface::~ActionSwigInterface(){
  engine().close(*S);
}


void ActionSwigInterface::Cancel(){
  //engine().cancel(*S);
  cout << S->quitSignal.get();
  S->quitSignal.set() = true;
}

stringV ActionSwigInterface::getShapeList(){
  stringV strs;
  std::stringstream tmp;
  S->modelWorld.readAccess();
  for(ors::Shape *shape: S->modelWorld().shapes){
    tmp.str(""),
    tmp.clear();
    tmp <<shape->name;
    strs.push_back(tmp.str());
  }
  S->modelWorld.deAccess();
  return strs;
}

stringV ActionSwigInterface::getBodyList(){
  stringV strs;
  std::stringstream tmp;
  S->modelWorld.readAccess();
  for(ors::Body *body: S->modelWorld().bodies){
    tmp.str(""),
    tmp.clear();
    tmp << body->name;
    strs.push_back(tmp.str());
  }
  S->modelWorld.deAccess();
  return strs;
}

stringV ActionSwigInterface::getJointList(){
  stringV strs;
  std::stringstream tmp;
  S->modelWorld.readAccess();
  for(ors::Joint *joint: S->modelWorld().joints){
    tmp.str(""),
    tmp.clear();
    tmp << joint->name;
    strs.push_back(tmp.str());
  }
  S->modelWorld.deAccess();
  return strs;
}

dict ActionSwigInterface::getBodyByName(std::string bodyName){
  dict D;
  S->modelWorld.readAccess();
  ors::Body *body = S->modelWorld().getBodyByName(bodyName.c_str());
  D["name"]= bodyName;
  D["type"] = std::to_string(body->type);
  D["Q"] =  STRING('[' <<body->X.rot<<']');
  D["pos"] = STRING('[' <<body->X.pos<<']');
  S->modelWorld.deAccess();
  return D;
}

dict ActionSwigInterface::getJointByName(std::string jointName){
  dict D;
  S->modelWorld.readAccess();
  ors::Joint *joint = S->modelWorld().getJointByName(jointName.c_str());
  D["name"]= jointName;
  D["type"] = std::to_string(joint->type);
  D["Q"] =  STRING('[' <<joint->X.rot<<']');
  D["pos"] = STRING('[' <<joint->X.pos<<']');
  S->modelWorld.deAccess();
  return D;
}

dict ActionSwigInterface::getShapeByName(std::string shapeName){
  dict D;
  S->modelWorld.readAccess();
  ors::Shape *shape = S->modelWorld().getShapeByName(shapeName.c_str());
  D["name"]= shapeName;
  D["type"] = std::to_string(shape->type);
  D["Q"] =  STRING('[' <<shape->X.rot<<']');
  D["pos"] = STRING('[' <<shape->X.pos<<']');
  S->modelWorld.deAccess();
  return D;
}


int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Node *symbol = S->RM.get()->KB.getNode(symbolName.c_str());
  CHECK(symbol,"The symbol name '" <<symbolName <<"' is not defined");
  return symbol->index;
}

intV ActionSwigInterface::str2lit(stringV symbolNames){
  intV I;
  for(auto& s:symbolNames) I.push_back(getSymbolInteger(s));
  return I;
}

stringV ActionSwigInterface::lit2str(intV literals){
  stringV strs;
  for(auto i:literals) strs.push_back(S->RM.get()->KB.elem(i)->keys.last().p);
  return strs;
}

bool  ActionSwigInterface::isTrue(const stringV& literals){
  return S->RM.get()->queryCondition(lits2str(literals));
}

void ActionSwigInterface::setFact(const char* fact){
  S->effects.set()() <<fact <<", ";
  S->state.waitForNextRevision(); //TODO: is this robust?
}

void ActionSwigInterface::stopFact(const char* fact){
  S->effects.set()() <<fact <<"!, ";
  S->state.waitForNextRevision(); //TODO: is this robust?
}

stringV ActionSwigInterface::getFacts(){
  stringV V;
  std::stringstream tmp;
  for(auto i:*S->RM.set()->state){
    tmp.str(""),
    tmp.clear();
    tmp << *i;
    V.push_back(tmp.str());

  }
  return V;
}

void ActionSwigInterface::startActivity(const stringV& literals, const dict& parameters){
  setFact(lits2str(literals, parameters));
}

void ActionSwigInterface::stopActivity(const stringV& literals){
  stopFact(lits2str(literals));
}

void ActionSwigInterface::waitForCondition(const stringV& literals){
  for(;;){
    if(isTrue(literals)) return;
    S->state.waitForNextRevision();
  }
}

void ActionSwigInterface::waitForCondition(const char* query){
  for(;;){
    if(S->RM.get()->queryCondition(query)) return;
    S->state.waitForNextRevision();
  }
}

int ActionSwigInterface::waitForOrCondition(const std::vector<stringV> literals){
  for(;;){
    for(unsigned i=0; i < literals.size(); i++){
      if(isTrue(literals[i])) return i;
    }
    S->state.waitForNextRevision();
  }

}
//void ActionSwigInterface::startActivity(intV literal, const dict& parameters){
//#if 1
//  startActivity(lit2str(literal), parameters);
//#else
//  S->RM.writeAccess();
//  for(auto i:literal) parents.append(S->RM().elem(i));
//  state.append<bool>({}, parents, NULL, false);
//  S->RM.deAccess();
//#endif
//}

//void ActionSwigInterface::waitForCondition(intV literals){
//#if 1
//  waitForCondition(lit2str(literals));
//#else
//  S->RM.readAccess();
//  for(auto i:literal) lit.append(S->RM().elem(i));
//  S->RM.deAccess();

//  bool cont = true;
//  while (cont) {
//    S->RM.waitForNextRevision();
//    S->RM.readAccess();
//    Item *it = getEqualFactInKB(state, lit);
//    if(it) cont=false;
//    S->RM.deAccess();
//  }
//#endif
//}

void ActionSwigInterface::waitForQuitSymbol(){
  waitForCondition(stringV({"quit"}));
}

int ActionSwigInterface::createNewSymbol(std::string symbolName){
#if 1
  Node *symbol = S->RM.set()->declareNewSymbol(symbolName.c_str());
#else
  Item *symbol = S->RM.set()->append<bool>(symbolName.c_str(), NULL, false);
#endif
  return symbol->index;
}

stringV ActionSwigInterface::getSymbols() {
  stringV V;
  std::stringstream tmp;
  for (auto* i:S->RM.get()->KB){
    tmp.str(""),
    tmp.clear();
    tmp << *i;//->keys(0);
    V.push_back(tmp.str());
  }
  return V;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, const stringV& parentSymbols, const dict& parameters){
#if 1
  MT::String str;
  str <<symbolName.c_str() <<lits2str(parentSymbols, parameters);
  Node *symbol = S->RM.set()->declareNewSymbol(str);
#else
  S->RM.writeAccess();

  Item *symbol = S->RM().append<bool>(symbolName.c_str(), NULL, false);
  Graph *td = new Graph(parameters);
  S->RM().append<Graph>({"Task"}, {symbol}, td, true);
  S->RM().checkConsistency();
  //cout <<S->RM() <<endl;
  S->RM.deAccess();
  s->activity.machine->parseTaskDescription(*td);
#endif
  return symbol->index;
}

void ActionSwigInterface::execScript(const char* filename){
  FILE(filename) >>S->RM.set()->KB;

  Node *s = S->RM.get()->KB.getNode("Script");
  Graph& script = s->graph();
  int rev=0;
  for(Node* n:script){
    if(n->parents.N==0 && n->getValueType()==typeid(Graph)){ //interpret as wait
      for(;;){
        if(allFactsHaveEqualsInScope(*S->RM.get()->state, n->graph())) break;
        rev=S->RM.waitForRevisionGreaterThan(rev);
      }
    }else{ //interpret as set fact
//      applySubstitutedLiteral(*S->RM.set()->state, n, {}, NULL);
      S->RM.set()->applyEffect(n, true);
//      S->rmm->threadStep();
//      S->effects.set()() <<"(go)"; //just trigger that the RM module steps
    }
  }
}

