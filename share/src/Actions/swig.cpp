#include "swig.h"

//#include <Actions/actionMachine.h>
//#include <Actions/actionMachine_internal.h>
//#include <Actions/actions.h>
//#include <Motion/motionHeuristics.h>
#include <FOL/fol.h>
#include <Ors/ors.h>
#include <Actions/TaskControllerModule.h>
#include <FOL/relationalMachineModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>

// ============================================================================
struct SwigSystem : System{
  ACCESS(bool, quitSignal)
  ACCESS(RelationalMachine, RM)
  ACCESS(MT::String, effects)
  ACCESS(MT::String, state)
  TaskControllerModule *tcm;
  SwigSystem(){
    tcm = addModule<TaskControllerModule>(NULL, Module::loopWithBeat, .01);
    addModule<ActivitySpinnerModule>(NULL, Module::loopWithBeat, .01);
    addModule<RelationalMachineModule>(NULL, Module::listenFirst, .01);

    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
//      addModule<RosCom_ForceSensorSync>(NULL, Module::loopWithBeat, 1.);
    }
    connect();
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

ActionSwigInterface::ActionSwigInterface(bool useRos){
  S = new SwigSystem();
  engine().open(*S, true);

  taskControllerModule()->verbose=false;

  createNewSymbol("conv");
  createNewSymbol("contact");
  createNewSymbol("timeout");
//  new CoreTasks(*s->activity.machine);

//  S->RM.writeAccess();
//  S->RM().KB.append<Graph>({"STATE"}, {}, new Graph(), true);
//  S->RM().KB.checkConsistency();
//  S->RM.deAccess();
}

ActionSwigInterface::~ActionSwigInterface(){
  engine().close(*S);
}

dict ActionSwigInterface::getBodyByName(std::string bodyName){
  dict D;
  S->tcm->mutex.lock();
  ors::Body *body = S->tcm->modelWorld.getBodyByName(bodyName.c_str());
  D["pos"] = STRING('[' <<body->X.pos <<']');
  S->tcm->mutex.unlock();
  return D;
}

dict ActionSwigInterface::getShapeByName(std::string shapeName){
  dict D;
  S->tcm->mutex.lock();
  ors::Shape *shape = S->tcm->modelWorld.getShapeByName(shapeName.c_str());
  D["pos"] = STRING('[' <<shape->X.pos <<']');
  S->tcm->mutex.unlock();
  return D;
}

dict ActionSwigInterface::getJointByName(std::string jointName){
  dict D;
  S->tcm->mutex.lock();
  ors::Joint *joint = S->tcm->modelWorld.getJointByName(jointName.c_str());
  D["pos"] = STRING('[' <<joint->X.pos <<']');
  S->tcm->mutex.unlock();
  return D;
}

int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Item *symbol = S->RM.get()->KB.getItem(symbolName.c_str());
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

void ActionSwigInterface::setFact(const char* fact){
  S->effects.set()() <<fact <<", ";
  S->state.waitForNextRevision();
}

void ActionSwigInterface::startActivity(const stringV& literals, const dict& parameters){
  setFact(lits2str(literals, parameters));
}

void ActionSwigInterface::waitForCondition(const char* query){
  for(;;){
    if(S->RM.get()->queryCondition(query)) return;
    S->state.waitForNextRevision();
  }
}

void ActionSwigInterface::waitForCondition(const stringV& literals){
  waitForCondition(lits2str(literals));
}

//void ActionSwigInterface::startActivity(intV literal, const dict& parameters){
//#if 1
//  startActivity(lit2str(literal), parameters);
//#else
//  S->RM.writeAccess();
//  Graph& state=S->RM().getItem("STATE")->kvg();
//  ItemL parents;
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
//  Graph& state=S->RM().getItem("STATE")->kvg();
//  ItemL lit;
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
  Item *symbol = S->RM.set()->declareNewSymbol(symbolName.c_str());
#else
  Item *symbol = S->RM.set()->append<bool>(symbolName.c_str(), NULL, false);
#endif
  return symbol->index;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, const stringV& parentSymbols, const dict& parameters){
#if 1
  MT::String str;
  str <<symbolName.c_str() <<lits2str(parentSymbols, parameters);
  Item *symbol = S->RM.set()->declareNewSymbol(str);
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

