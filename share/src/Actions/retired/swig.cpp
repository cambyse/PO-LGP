#include "swig.h"

#include <Actions/actionMachine.h>
//#include <Actions/actionMachine_internal.h>
//#include <Actions/actions.h>
//#include <Motion/motionHeuristics.h>
#include <FOL/fol.h>
#include <Ors/ors.h>

struct sActionSwigInterface{
  ActionSystem activity;
  Access_typed<Graph>& KB;
  sActionSwigInterface():activity(), KB(activity.machine->KB){}
};

ActionSwigInterface::ActionSwigInterface(bool useRos){
  s = new sActionSwigInterface();
  engine().open(s->activity);

  createNewSymbol("conv");
  createNewSymbol("contact");
  createNewSymbol("timeout");
//  new CoreTasks(*s->activity.machine);

  s->KB.writeAccess();
  s->KB().append<Graph>({"STATE"}, {}, new Graph(), true);
  s->KB().checkConsistency();
  s->KB.deAccess();

}

ActionSwigInterface::~ActionSwigInterface(){
  engine().close(s->activity);
}

dict ActionSwigInterface::getBodyByName(std::string bodyName){
dict D;
ors::Body *body = s->activity.machine->world->getBodyByName(bodyName.c_str());
D["pos"] = "[" + std::to_string(body->X.pos.x) + ", " + std::to_string(body->X.pos.y) + ", " + std::to_string(body->X.pos.z) + "]";
return D;
}

dict ActionSwigInterface::getShapeByName(std::string shapeName){
dict D;
ors::Shape *shape = s->activity.machine->world->getShapeByName(shapeName.c_str());
D["pos"] = "[" + std::to_string(shape->X.pos.x) + ", " + std::to_string(shape->X.pos.y) + ", " + std::to_string(shape->X.pos.z) + "]";
return D;
}

dict ActionSwigInterface::getJointByName(std::string jointName){
dict D;
ors::Joint *joint = s->activity.machine->world->getJointByName(jointName.c_str());
D["pos"] = "[" + std::to_string(joint->X.pos.x) + ", " + std::to_string(joint->X.pos.y) + ", " + std::to_string(joint->X.pos.z) + "]";
return D;
}

int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Node *symbol = s->KB.get()->getItem(symbolName.c_str());
  CHECK(symbol,"The symbol name '" <<symbolName <<"' is not defined");
  return symbol->index;
}

intV ActionSwigInterface::lit(stringV symbolNames){
  intV I;
  for(auto& s:symbolNames) I.push_back(getSymbolInteger(s));
  return I;
}


void ActionSwigInterface::startActivity(intV literal, dict parameters){
  s->KB.writeAccess();
  Graph& state=s->KB().getItem("STATE")->graph();
  NodeL parents;
  for(auto i:literal) parents.append(s->KB().elem(i));
  state.append<bool>({}, parents, NULL, false);
  s->KB.deAccess();
}

void ActionSwigInterface::waitForCondition(intV literal){
  s->KB.readAccess();
  Graph& state=s->KB().getItem("STATE")->graph();
  NodeL lit;
  for(auto i:literal) lit.append(s->KB().elem(i));
  s->KB.deAccess();

  bool cont = true;
  while (cont) {
    s->KB.waitForNextRevision();
    s->KB.readAccess();
    Node *it = getEqualFactInKB(state, lit);
    if(it) cont=false;
    s->KB.deAccess();
  }
}

void ActionSwigInterface::waitForQuitSymbol(){
  s->activity.machine->waitForQuitSymbol();
}

int ActionSwigInterface::createNewSymbol(std::string symbolName){
  Node *symbol = s->KB.set()->append<bool>(symbolName.c_str(), NULL, false);
  return symbol->index;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, dict parameters){
  s->KB.writeAccess();

  Node *symbol = s->KB().append<bool>(symbolName.c_str(), NULL, false);
  Graph *td = new Graph(parameters);
  s->KB().append<Graph>({"Task"}, {symbol}, td, true);
  s->KB().checkConsistency();
  //cout <<s->KB() <<endl;
  s->KB.deAccess();
  s->activity.machine->parseTaskDescription(*td);
  return symbol->index;
}

