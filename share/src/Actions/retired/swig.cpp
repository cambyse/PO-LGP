#include "swig.h"

#include <Actions/actionMachine.h>
//#include <Actions/actionMachine_internal.h>
//#include <Actions/actions.h>
//#include <Motion/motionHeuristics.h>
#include <Logic/fol.h>
#include <Kin/kin.h>

struct sActionSwigInterface{
  ActionSystem activity;
  Access<Graph>& KB;
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
  s->KB().newNode<Graph*>({"STATE"}, {}, new Graph());
  s->KB().checkConsistency();
  s->KB.deAccess();

}

ActionSwigInterface::~ActionSwigInterface(){
  engine().close(s->activity);
}

dict ActionSwigInterface::getBodyByName(std::string bodyName){
dict D;
mlr::Body *body = s->activity.machine->world->getBodyByName(bodyName.c_str());
D["pos"] = "[" + std::to_string(body->X.pos.x) + ", " + std::to_string(body->X.pos.y) + ", " + std::to_string(body->X.pos.z) + "]";
return D;
}

dict ActionSwigInterface::getShapeByName(std::string shapeName){
dict D;
mlr::Shape *shape = s->activity.machine->world->getShapeByName(shapeName.c_str());
D["pos"] = "[" + std::to_string(shape->X.pos.x) + ", " + std::to_string(shape->X.pos.y) + ", " + std::to_string(shape->X.pos.z) + "]";
return D;
}

dict ActionSwigInterface::getJointByName(std::string jointName){
dict D;
mlr::Joint *joint = s->activity.machine->world->getJointByName(jointName.c_str());
D["pos"] = "[" + std::to_string(joint->X.pos.x) + ", " + std::to_string(joint->X.pos.y) + ", " + std::to_string(joint->X.pos.z) + "]";
return D;
}

int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Node *symbol = s->KB.get()->getNode(symbolName.c_str());
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
  Graph& state=s->KB().get<Graph>("STATE");
  NodeL parents;
  for(auto i:literal) parents.append(s->KB().elem(i));
  state.newNode<bool>({}, parents, NULL, false);
  s->KB.deAccess();
}

void ActionSwigInterface::waitForCondition(intV literal){
  s->KB.readAccess();
  Graph& state=s->KB().get<Graph>("STATE");
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
  Node *symbol = s->KB.set()->newNode<bool>(symbolName.c_str(), NULL, false);
  return symbol->index;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, dict parameters){
  s->KB.writeAccess();

  Node *symbol = s->KB().newNode<bool>(symbolName.c_str(), NULL, false);
  Graph *td = new Graph(parameters);
  s->KB().newNode<Graph*>({"Task"}, {symbol}, td);
  s->KB().checkConsistency();
  //cout <<s->KB() <<endl;
  s->KB.deAccess();
  s->activity.machine->parseTaskDescription(*td);
  return symbol->index;
}

