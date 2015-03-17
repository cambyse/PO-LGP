#include "swig.h"

#include <Actions/actionMachine.h>
//#include <Actions/actionMachine_internal.h>
//#include <Actions/actions.h>
//#include <Motion/motionHeuristics.h>
#include <FOL/fol.h>

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
  s->KB().append<Graph>("STATE", new Graph(), true);
  s->KB().checkConsistency();
  s->KB.deAccess();

}

ActionSwigInterface::~ActionSwigInterface(){
  engine().close(s->activity);
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
  //cout <<s->KB() <<endl;
  s->KB.deAccess();
  s->activity.machine->parseTaskDescription(*td);
  return symbol->index;
}

