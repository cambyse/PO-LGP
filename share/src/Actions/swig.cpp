#include "swig.h"

#include <Actions/actionMachine.h>
//#include <Actions/actionMachine_internal.h>
//#include <Actions/actions.h>
//#include <Motion/motionHeuristics.h>
#include <FOL/fol.h>

struct sActionSwigInterface{
  ActionSystem activity;
};

ActionSwigInterface::ActionSwigInterface(bool useRos){
  s = new sActionSwigInterface;
  engine().open(s->activity);

  createNewSymbol("conv");
  createNewSymbol("contact");
  createNewSymbol("timeout");
//  new CoreTasks(*s->activity.machine);

}

ActionSwigInterface::~ActionSwigInterface(){
  engine().close(s->activity);
}

int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Item *symbol = s->activity.machine->KB.get()->getItem(symbolName.c_str());
  return symbol->index;
}

intV ActionSwigInterface::lit(stringV symbolNames){
  intV I;
  for(auto& s:symbolNames) I.push_back(getSymbolInteger(s));
  return I;
}


void ActionSwigInterface::startActivity(intV literal, dict parameters){
  s->activity.machine->KB.writeAccess();
  Graph& KB=s->activity.machine->KB();

  ItemL parents;
  for(auto i:literal) parents.append(KB(i));
  KB.append<bool>(STRINGS_0(), parents, NULL, false);
  s->activity.machine->KB.deAccess();
}

void ActionSwigInterface::waitForCondition(intV literal){
  auto& KB=s->activity.machine->KB;
  KB.readAccess();
  ItemL lit;
  for(auto i:literal) lit.append(KB()(i));
  KB.deAccess();

  bool cont = true;
  while (cont) {
    KB.waitForNextRevision();
    KB.readAccess();
    Item *it = getEqualFactInKB(KB(), lit);
    if(it) cont=false;
    KB.deAccess();
  }
}

void ActionSwigInterface::waitForQuitSymbol(){
  s->activity.machine->waitForQuitSymbol();
}

int ActionSwigInterface::createNewSymbol(std::string symbolName){
  Item *symbol = s->activity.machine->KB.set()->append<bool>(symbolName.c_str(), NULL, false);
  return symbol->index;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, dict parameters){
  s->activity.machine->KB.writeAccess();
  Graph& KB=s->activity.machine->KB();

  Item *symbol = KB.append<bool>(symbolName.c_str(), NULL, false);
  Graph *td = new Graph(parameters);
  KB.append<Graph>(STRINGS_1("Task"), {symbol}, td, true);
  cout <<KB <<endl;
  s->activity.machine->KB.deAccess();
  s->activity.machine->parseTaskDescription(*td);
  return symbol->index;
}

