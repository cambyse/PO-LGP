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

stringV ActionSwigInterface::getShapeList(){
  stringV S;
  std::stringstream tmp;
  for(ors::Shape *shape: s->activity.machine->world->shapes){
    tmp.str(""),
    tmp.clear();
    tmp << shape->name;
    S.push_back(tmp.str());
  }
  return S;
}

stringV ActionSwigInterface::getBodyList(){
  stringV S;
  std::stringstream tmp;
  for(ors::Body *body: s->activity.machine->world->bodies){
    tmp.str(""),
    tmp.clear();
    tmp << body->name;
    S.push_back(tmp.str());
  }
  return S;
}

stringV ActionSwigInterface::getJointList(){
  stringV S;
  std::stringstream tmp;
  for(ors::Joint *joint: s->activity.machine->world->joints){
    tmp.str(""),
    tmp.clear();
    tmp << joint->name;
    S.push_back(tmp.str());
  }
  return S;
}

dict ActionSwigInterface::getBodyByName(std::string bodyName){
dict D;
ors::Body *body = s->activity.machine->world->getBodyByName(bodyName.c_str());
D["pos"] = "[" + std::to_string(body->X.pos.x) + ", " + std::to_string(body->X.pos.y) + ", " + std::to_string(body->X.pos.z) + "]";
D["name"]= bodyName;
D["type"] = std::to_string(body->type);
D["Q"] = "[" + std::to_string(body->X.rot.w) + ", " + std::to_string(body->X.pos.x) + ", " + std::to_string(body->X.pos.y) + ", " + std::to_string(body->X.pos.z) + "]";
cout << std::to_string(body->X.pos.x) << endl;
return D;
}

dict ActionSwigInterface::getShapeByName(std::string shapeName){
dict D;
ors::Shape *shape = s->activity.machine->world->getShapeByName(shapeName.c_str());
D["pos"] = "[" + std::to_string(shape->X.pos.x) + ", " + std::to_string(shape->X.pos.y) + ", " + std::to_string(shape->X.pos.z) + "]";
D["name"]= shapeName;
D["type"] = std::to_string(shape->type);
D["Q"] = "[" + std::to_string(shape->X.rot.w) + ", " + std::to_string(shape->X.pos.x) + ", " + std::to_string(shape->X.pos.y) + ", " + std::to_string(shape->X.pos.z) + "]";
return D;
}

dict ActionSwigInterface::getJointByName(std::string jointName){
dict D;
ors::Joint *joint = s->activity.machine->world->getJointByName(jointName.c_str());
D["pos"] = "[" + std::to_string(joint->X.pos.x) + ", " + std::to_string(joint->X.pos.y) + ", " + std::to_string(joint->X.pos.z) + "]";
D["name"]= jointName;
D["type"] = std::to_string(joint->type);
D["Q"] = "[" + std::to_string(joint->X.rot.w) + ", " + std::to_string(joint->X.pos.x) + ", " + std::to_string(joint->X.pos.y) + ", " + std::to_string(joint->X.pos.z) + "]";
return D;
}

int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Item *symbol = s->KB.get()->getItem(symbolName.c_str());
  CHECK(symbol,"The symbol name '" <<symbolName <<"' is not defined");
  return symbol->index;
}

intV ActionSwigInterface::lit(stringV symbolNames){
  intV I;
  for(auto& s:symbolNames) I.push_back(getSymbolInteger(s));
  return I;
}

stringV ActionSwigInterface::getSymbols(){
  stringV S;
  std::stringstream tmp;
  s->KB.readAccess();
  for (auto& i:s->KB()){
    tmp.str(""),
    tmp.clear();
    tmp << i->keys(0);
    S.push_back(tmp.str());
  }
  s->KB.deAccess();
  return S;
}

void ActionSwigInterface::startActivity(intV literal, dict parameters){
  s->KB.writeAccess();
  Graph& state=s->KB().getItem("STATE")->kvg();
  ItemL parents;
  for(auto i:literal) parents.append(s->KB().elem(i));
  state.append<bool>({}, parents, NULL, false);
  s->KB.deAccess();
}

void ActionSwigInterface::waitForCondition(intV literal){
  s->KB.readAccess();
  Graph& state=s->KB().getItem("STATE")->kvg();
  ItemL lit;
  for(auto i:literal) lit.append(s->KB().elem(i));
  s->KB.deAccess();

  bool cont = true;
  while (cont) {
    s->KB.waitForNextRevision();
    s->KB.readAccess();
    Item *it = getEqualFactInKB(state, lit);
    if(it) cont=false;
    s->KB.deAccess();
  }
}

intV ActionSwigInterface::getStateLiterals(){
  intV I;
  s->KB.readAccess();
  Item* state=s->KB().getItem("STATE");
  cout << s->KB()<< endl;
  return I;
}

void ActionSwigInterface::waitForQuitSymbol(){
  s->activity.machine->waitForQuitSymbol();
}

int ActionSwigInterface::createNewSymbol(std::string symbolName){
  Item *symbol = s->KB.set()->append<bool>(symbolName.c_str(), NULL, false);
  return symbol->index;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, dict parameters){
  s->KB.writeAccess();

  Item *symbol = s->KB().append<bool>(symbolName.c_str(), NULL, false);
  Graph *td = new Graph(parameters);
  s->KB().append<Graph>({"Task"}, {symbol}, td, true);
  s->KB().checkConsistency();
  s->KB.deAccess();
  s->activity.machine->parseTaskDescription(*td);
  return symbol->index;
}

